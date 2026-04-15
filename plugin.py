"""
Smart Virtual Thermostat python plugin for Domoticz
Author: Logread,
        adapted from the Vera plugin by Antor, see:
            http://www.antor.fr/apps/smart-virtual-thermostat-eng-2/?lang=en
            https://github.com/AntorFr/SmartVT
Version: 0.4.23 (April 2026) - see history.txt for versions history

Changes in 0.4.23:
    - Fix: ConstC recovery now triggers when ConstC > 120 (instead of > 150) and setpoint is
      reached or exceeded. Previously ConstC could stay in the 120-150 range indefinitely,
      producing anomalous power calculations and spurious short heating cycles in spring/autumn
      when less heating power is needed than in winter.

Changes in 0.4.22:
    - Improved: updateBeta() now logs each day used in the calculation (date, max, min, range)
      and the resulting average in Verbose mode, for easier diagnostics.

Changes in 0.4.21:
    - Fix: lastcalc initialized to None in __init__ and checked before use in AutoCallib to avoid
      a near-zero timedelta on the very first cycle causing anomalous ConstC/ConstT calculations.
    - Fix: self.heat explicitly set to False in pause activation and forced mode transitions
      to prevent it remaining True if switchHeat() returns early (no heaters found).
    - Fix: isValveOpen() and switchHeat() now use getdevices&rid=<idx> for single-device queries
      instead of downloading the full device list, reducing API call overhead.
    - Fix: updateBeta() skips the current (incomplete) day and uses the last 7 complete days
      to avoid using partial daily data that would skew the beta calculation.
    - Fix: AutoCallib guards against lastcalc==now (zero timedelta) to prevent ConstC/ConstT
      being driven toward zero by a division producing 0.

Changes in 0.4.20:
    - Fix: valve wait state (heatduration_pending, valveopen, valveWaitStart) is now reset in all
      cases where heating is interrupted: thermostat Off, Pause activation, Forced mode on/off,
      and any onCommand that forces recalculation. Previously, heatduration_pending could remain
      nonzero (zombie state) blocking the plugin in "Waiting for valve" loop indefinitely.

Changes in 0.4.19:
    - Fix: on plugin restart, pause state is now restored from the Thermostat Pause device status.
      Previously, restarting the plugin while pause was active would cause the heater to turn on
      because internal pause state was always reset to False on startup.

Changes in 0.4.18:
    - Added dynamic beta calculation from external temperature history.
      Every 24h, fetches the last 7 days of external temperature data from Domoticz API
      and computes beta as the average daily temperature range (max - min).
      Beta reduces effective_nbCT when computing alpha_T, making ConstT learning faster
      during high thermal variability (autumn/winter swings) and slower when stable.
      If no external sensor is configured, beta stays 0.0 (behaviour unchanged).
      New persistent fields: 'beta' and 'LastBetaUpdate'.

Changes in 0.4.17:
    - Fix: ConstT learning now uses Exponential Moving Average (EMA) instead of additive delta.
      Previous formula used += delta which could only grow over time and never converge downward.
      EMA allows ConstT to decrease naturally when outside temperatures rise (e.g. spring/summer),
      symmetrically with how ConstC is already learned.
    - Fix: added safety clamp lower bounds: ConstC minimum 1.0, ConstT minimum 0.0.
      Previously only upper bounds were enforced (ConstC max 150, ConstT max 10).
    - Fix: if ConstT >= 10 (at maximum) and setpoint was reached, force ConstT down by 15% per cycle
      to recover from diverged state without manual intervention, same logic as ConstC recovery.

Changes in 0.4.16:
    - Added optional valve contact sensor support (7th value in Mode5): when configured, the heat
      timer starts only after the valve contact confirms the valve is open, not when SVT commands
      the heater switch. If not set or 0, behaviour is unchanged.
    - Fix: LastSetPoint now updated immediately in memory (and persisted) when setpoint changes,
      so AutoCallib uses the correct reference on the very next cycle.
    - Fix: AutoCallib now learns ConstC downward when LastPwr==100 but setpoint was reached/exceeded,
      breaking the divergence loop that caused ConstC to grow unbounded.
    - Fix: added safety clamp on ConstC (max 150) and ConstT (max 10) to prevent runaway values.
    - Fix: if ConstC > 150 and power was 100% but setpoint not reached, force ConstC down by 15%
      per cycle to recover from diverged state without manual intervention.

Changes in 0.4.15:
    - AutoCallib: replaced simple capped average with Exponential Moving Average (EMA)
      for faster adaptation to changes in room/heater characteristics
    - AutoMode: replaced hard 100% boost with proportional boost ramp (less overshoot)
    - AutoMode: added lightweight integral term (PI controller) to eliminate steady-state
      drift below setpoint; includes anti-windup clamp
    - getUserVar: replaced unsafe eval() with json.loads() for safe persistent variable loading;
      legacy Python dict format recovered via ast.literal_eval with immediate re-save as JSON
    - saveUserVar: now saves as JSON instead of Python repr string
    - except clauses narrowed from bare except to except Exception throughout
    - parseCSV: simplified, integers only; deltamax parsed separately as float
    - Logging: all debug/info calls now routed through WriteLog() consistently
    - Fix: version tag in XML header updated to match actual version
"""
"""
<plugin key="SVT" name="Smart Virtual Thermostat" author="logread" version="0.4.23" wikilink="https://www.domoticz.com/wiki/Plugins/Smart_Virtual_Thermostat.html" externallink="https://github.com/999LV/SmartVirtualThermostat.git">
    <description>
        <h2>Smart Virtual Thermostat</h2><br/>
        Easily implement in Domoticz an advanced virtual thermostat based on time modulation<br/>
        and self learning of relevant room thermal characteristics (including insulation level)<br/>
        rather then more conventional hysteresis methods, so as to achieve a greater comfort.<br/>
        It is a port to Domoticz of the original Vera plugin from Antor.<br/>
        <h3>Set-up and Configuration</h3>
        See domoticz wiki above.<br/> 
    </description>
    <params>
        <param field="Address" label="Domoticz IP Address" width="200px" required="true" default="localhost"/>
        <param field="Port" label="Port" width="40px" required="true" default="8080"/>
        <param field="Username" label="Username" width="200px" required="false" default=""/>
        <param field="Password" label="Password" width="200px" required="false" default=""/>
        <param field="Mode1" label="Inside Temperature Sensors (csv list of idx)" width="100px" required="true" default="0"/>
        <param field="Mode2" label="Outside Temperature Sensors (csv list of idx)" width="100px" required="false" default=""/>
        <param field="Mode3" label="Heating Switches (csv list of idx)" width="100px" required="true" default="0"/>
        <param field="Mode4" label="Apply minimum heating per cycle" width="200px">
            <options>
		<option label="only when heating required" value="Normal"  default="true" />
                <option label="always" value="Forced"/>
            </options>
        </param> 
        <param field="Mode5" label="Calc. cycle, Min. Heating time /cycle, Pause On delay, Pause Off delay, Forced mode duration (all in minutes), Delta max (°C), Valve contact sensor idx (0=disabled)" width="250px" required="true" default="30,0,2,1,60,0.2,0"/>
        <param field="Mode6" label="Logging Level" width="200px">
            <options>
                <option label="Normal" value="Normal"  default="true"/>
                <option label="Verbose" value="Verbose"/>
                <option label="Debug - Python Only" value="2"/>
                <option label="Debug - Basic" value="62"/>
                <option label="Debug - Basic+Messages" value="126"/>
                <option label="Debug - Connections Only" value="16"/>
                <option label="Debug - Connections+Queue" value="144"/>
                <option label="Debug - All" value="-1"/>
            </options>
        </param>
    </params>
</plugin>
"""
import Domoticz
import json
from urllib import parse, request
from datetime import datetime, timedelta
import time
import base64
import itertools


class deviceparam:

    def __init__(self, unit, nvalue, svalue):
        self.unit = unit
        self.nvalue = nvalue
        self.svalue = svalue


class BasePlugin:

    def __init__(self):

        self.debug = False
        self.calculate_period = 30  # Time in minutes between two calculations (cycle)
        self.minheatpower = 0  # if heating is needed, minimum heat power (in % of calculation period)
        self.deltamax = 0.2  # allowed temp excess over setpoint temperature
        self.pauseondelay = 2  # time between pause sensor actuation and actual pause
        self.pauseoffdelay = 1  # time between end of pause sensor actuation and end of actual pause
        self.forcedduration = 60  # time in minutes for the forced mode
        self.ActiveSensors = {}
        self.InTempSensors = []
        self.OutTempSensors = []
        self.Heaters = []
        self.InternalsDefaults = {
            'ConstC': float(60),  # inside heating coeff, depends on room size & power of your heater (60 by default)
            'ConstT': float(1),  # external heating coeff, depends on insulation relative to the outside (1 by default)
            'nbCC': 0,  # number of learnings for ConstC
            'nbCT': 0,  # number of learnings for ConstT
            'LastPwr': 0,  # % power from last calculation
            'LastInT': float(0),  # inside temperature at last calculation
            'LastOutT': float(0),  # outside temperature at last calculation
            'LastSetPoint': float(20),  # setpoint at time of last calculation
            'ALStatus': 0,  # AutoLearning status (0 = uninitialized, 1 = initialized, 2 = disabled)
            'beta': 0.0,          # [NEW] average daily external temp range (max-min) over last 7 days
            'LastBetaUpdate': ""}  # [NEW] date of last beta update (YYYY-MM-DD string)
        self.Internals = self.InternalsDefaults.copy()
        self.heat = False
        self.pause = False
        self.pauserequested = False
        self.pauserequestchangedtime = datetime.now()
        self.forced = False
        self.boost = True        # boost heating when boostgap is reached
        self.boostgap = 0.5      # gap in °C between inside temp and setpoint above which boost mode activates
        self.boostmaxpower = 85  # [IMPROVED] max boost power (was hardcoded 100%). Leaves headroom to avoid overshoot.
        self.intemp = 20.0
        self.outtemp = 20.0
        self.setpoint = 20.0
        self.endheat = datetime.now()
        self.nextcalc = self.endheat
        self.lastcalc = None  # [FIX] None until first AutoMode() call; guards against zero timedelta in AutoCallib
        self.nextupdate = self.endheat
        self.nexttemps = self.endheat
        self.learn = True
        self.loglevel = None
        self.intemperror = False
        self.versionsupported = False

        # [NEW] Integral term state for PI controller.
        # Accumulates setpoint error over cycles to correct steady-state drift below setpoint.
        # Ki is intentionally small and conservative to nudge gently without instability.
        # integral_error is clamped (anti-windup) to avoid excessive accumulation during long cold spells.
        self.integral_error = 0.0
        self.Ki = 0.1           # integral gain
        self.integral_max = 20  # anti-windup clamp: integral contribution never exceeds ±20% power

        # [NEW] Valve contact sensor support (7th value in Mode5)
        # If configured, the heat timer starts only after the valve contact goes On (valve open).
        # valveidx = 0 means disabled (default behaviour).
        # valveopen = True means the valve contact has confirmed open since last heat command.
        # valveWaitStart = time when we started waiting for the valve to open.
        self.valveidx = 0
        self.valveopen = False
        self.valveWaitStart = None
        self.heatduration_pending = 0  # heat duration (minutes) waiting for valve to open

        return


    def onStart(self):

        # setup the appropriate logging level
        try:
            debuglevel = int(Parameters["Mode6"])
        except ValueError:
            debuglevel = 0
            self.loglevel = Parameters["Mode6"]
        if debuglevel != 0:
            self.debug = True
            Domoticz.Debugging(debuglevel)
            DumpConfigToLog()
            self.loglevel = "Verbose"
        else:
            self.debug = False
            Domoticz.Debugging(0)

        # check if the host domoticz version can run the plugin
        versionstr = Parameters["DomoticzVersion"]
        version = float(versionstr.split(" ")[0])
        if version >= 2023.2:
            self.versionsupported = True
        else:
            Domoticz.Error("Minimum domoticz version is 2023.2")
            return

        # create the child devices if these do not exist yet
        devicecreated = []
        if 1 not in Devices:
            Options = {"LevelActions": "||",
                       "LevelNames": "Off|Auto|Forced",
                       "LevelOffHidden": "false",
                       "SelectorStyle": "0"}
            Domoticz.Device(Name="Thermostat Control", Unit=1, TypeName="Selector Switch", Switchtype=18, Image=15,
                            Options=Options, Used=1).Create()
            devicecreated.append(deviceparam(1, 0, "0"))  # default is Off state
        if 2 not in Devices:
            Options = {"LevelActions": "||",
                       "LevelNames": "Off|Normal|Economy",
                       "LevelOffHidden": "true",
                       "SelectorStyle": "0"}
            Domoticz.Device(Name="Thermostat Mode", Unit=2, TypeName="Selector Switch", Switchtype=18, Image=15,
                            Options=Options, Used=1).Create()
            devicecreated.append(deviceparam(2, 0, "10"))  # default is normal mode
        if 3 not in Devices:
            Domoticz.Device(Name="Thermostat Pause", Unit=3, TypeName="Switch", Image=9, Used=1).Create()
            devicecreated.append(deviceparam(3, 0, ""))  # default is Off
        if 4 not in Devices:
            Domoticz.Device(Name="Setpoint Normal", Unit=4, Type=242, Subtype=1, Used=1).Create()
            devicecreated.append(deviceparam(4, 0, "20"))  # default is 20 degrees
        if 5 not in Devices:
            Domoticz.Device(Name="Setpoint Economy", Unit=5, Type=242, Subtype=1, Used=1).Create()
            devicecreated.append(deviceparam(5, 0, "20"))  # default is 20 degrees
        if 6 not in Devices:
            Domoticz.Device(Name="Thermostat temp", Unit=6, TypeName="Temperature").Create()
            devicecreated.append(deviceparam(6, 0, "20"))  # default is 20 degrees

        # if any device has been created in onStart(), now is time to update its defaults
        for device in devicecreated:
            Devices[device.unit].Update(nValue=device.nvalue, sValue=device.svalue)

        # build lists of sensors and switches
        self.InTempSensors = parseCSV(Parameters["Mode1"])
        self.WriteLog("Inside Temperature sensors = {}".format(self.InTempSensors), "Verbose")
        self.OutTempSensors = parseCSV(Parameters["Mode2"])
        self.WriteLog("Outside Temperature sensors = {}".format(self.OutTempSensors), "Verbose")
        self.Heaters = parseCSV(Parameters["Mode3"])
        self.WriteLog("Heaters = {}".format(self.Heaters), "Verbose")

        # build dict of status of all temp sensors to be used when handling timeouts
        for sensor in itertools.chain(self.InTempSensors, self.OutTempSensors):
            self.ActiveSensors[sensor] = True

        # splits additional parameters
        # [IMPROVED] parseCSV now returns only integers; deltamax parsed separately as float
        params = parseCSV(Parameters["Mode5"])
        if len(params) >= 5:
            self.calculate_period = CheckParam("Calculation Period", params[0], 30)
            if self.calculate_period < 5:
                Domoticz.Error("Invalid calculation period parameter. Using minimum of 5 minutes !")
                self.calculate_period = 5
            self.minheatpower = CheckParam("Minimum Heating (%)", params[1], 0)
            if self.minheatpower > 100:
                Domoticz.Error("Invalid minimum heating parameter. Using maximum of 100% !")
                self.minheatpower = 100
            self.pauseondelay = CheckParam("Pause On Delay", params[2], 2)
            self.pauseoffdelay = CheckParam("Pause Off Delay", params[3], 0)
            self.forcedduration = CheckParam("Forced Mode Duration", params[4], 60)
            if self.forcedduration < 15:
                Domoticz.Error("Invalid forced mode duration parameter. Using minimum of 15 minutes !")
                self.forcedduration = 15
            # deltamax is always the 6th field and is a float - parse it directly from raw string
            rawparams = Parameters["Mode5"].split(",")
            if len(rawparams) > 5:
                try:
                    self.deltamax = float(rawparams[5])
                except ValueError:
                    Domoticz.Error("Delta max has invalid value, using default of 0.2")
                    self.deltamax = 0.2
            else:
                Domoticz.Error("Delta max missing in parameters. Add the field in the plugin configuration (default value=0.2)")
            # read optional valve contact sensor idx (7th value in Mode5)
            if len(rawparams) > 6:
                try:
                    self.valveidx = int(rawparams[6])
                except ValueError:
                    self.valveidx = 0
            else:
                self.valveidx = 0
            if self.valveidx > 0:
                self.WriteLog("Valve contact sensor idx = {}".format(self.valveidx), "Verbose")
            else:
                self.WriteLog("Valve contact sensor: disabled", "Verbose")
        else:
            Domoticz.Error("Error reading Mode5 parameters")

        # loads persistent variables from dedicated user variable
        # note: to reset the thermostat to default values (i.e. ignore all past learning),
        # just delete the relevant "<plugin name>-InternalVariables" user variable in Domoticz GUI and restart plugin
        self.getUserVar()

        # [FIX] restore pause state from device status in case plugin was restarted while pause was active
        if Devices[3].nValue == 1:
            self.pauserequested = True
            self.pause = True
            # set pauserequestchangedtime in the past so pause is effective immediately (no delay on restart)
            self.pauserequestchangedtime = datetime.now() - timedelta(minutes=self.pauseondelay + 1)
            self.WriteLog("Pause state restored from device status", "Status")

        # if mode = off then make sure actual heating is off just in case it was manually set to on
        if Devices[1].sValue == "0":
            self.switchHeat(False)


    def onStop(self):

        Domoticz.Debugging(0)


    def onCommand(self, Unit, Command, Level, Color):

        self.WriteLog("onCommand called for Unit {}: Command '{}', Level: {}".format(Unit, Command, Level), "Verbose")

        # if host domoticz version is not OK than do nothing
        if not self.versionsupported:
            return

        if Unit == 3:  # pause switch
            self.pauserequestchangedtime = datetime.now()
            svalue = ""
            if str(Command) == "On":
                nvalue = 1
                self.pauserequested = True
            else:
                nvalue = 0
                self.pauserequested = False
        else:
            nvalue = 1 if Level > 0 else 0
            svalue = str(Level)

        Devices[Unit].Update(nValue=nvalue, sValue=svalue)

        if Unit in (1, 2, 4, 5):  # force recalculation if control or mode or a setpoint changed
            self.nextcalc = datetime.now()
            self.learn = False
            # [IMPROVED] reset integral error on setpoint/mode change to avoid windup from previous state
            self.integral_error = 0.0
            # [FIX] reset valve wait state to avoid zombie heatduration_pending blocking the next cycle
            self.heatduration_pending = 0
            self.valveopen = False
            self.valveWaitStart = None
            # [FIX] update LastSetPoint immediately so AutoCallib uses the correct reference
            # on the very next cycle, without waiting for AutoMode() to save it
            if Unit in (4, 5):
                if Devices[2].sValue == "10":
                    self.Internals['LastSetPoint'] = float(Devices[4].sValue)
                else:
                    self.Internals['LastSetPoint'] = float(Devices[5].sValue)
                self.WriteLog("LastSetPoint updated immediately to {}".format(
                    self.Internals['LastSetPoint']), "Verbose")
                self.saveUserVar()
            self.onHeartbeat()


    def onHeartbeat(self):

        # if host domoticz version is not OK than do nothing
        if not self.versionsupported:
            return

        now = datetime.now()

        # fool proof checking.... based on users feedback
        if not all(device in Devices for device in (1, 2, 3, 4, 5, 6)):
            Domoticz.Error("one or more devices required by the plugin is/are missing, please check domoticz device creation settings and restart !")
            return

        if Devices[1].sValue == "0":  # Thermostat is off
            if self.forced or self.heat:  # thermostat setting was just changed so we kill the heating
                self.forced = False
                self.endheat = now
                self.WriteLog("Switching heat Off !", "Verbose")
                self.switchHeat(False)
            # [FIX] always reset valve wait state when thermostat is off
            self.heatduration_pending = 0
            self.valveopen = False
            self.valveWaitStart = None

        elif Devices[1].sValue == "20":  # Thermostat is in forced mode
            if self.forced:
                if self.endheat <= now:
                    self.forced = False
                    self.endheat = now
                    self.WriteLog("Forced mode Off !", "Verbose")
                    Devices[1].Update(nValue=1, sValue="10")  # set thermostat to normal mode
                    self.switchHeat(False)
            else:
                self.forced = True
                self.endheat = now + timedelta(minutes=self.forcedduration)
                self.WriteLog("Forced mode On !", "Verbose")
                self.heat = True  # [FIX] explicitly set heat flag when forced mode starts
                self.switchHeat(True)
                # [FIX] reset valve wait state when forced mode starts
                self.heatduration_pending = 0
                self.valveopen = False
                self.valveWaitStart = None

        else:  # Thermostat is in mode auto

            if self.forced:  # thermostat setting was just changed from "forced" so we kill the forced mode
                self.forced = False
                self.endheat = now
                self.nextcalc = now  # this will force a recalculation on next heartbeat
                self.WriteLog("Forced mode Off !", "Verbose")
                self.switchHeat(False)
                # [FIX] reset valve wait state when forced mode ends
                self.heatduration_pending = 0
                self.valveopen = False
                self.valveWaitStart = None

            elif (self.endheat <= now or self.pause) and self.heat:  # heat cycle is over
                self.endheat = now
                self.heat = False
                self.valveopen = False
                self.heatduration_pending = 0
                self.valveWaitStart = None
                if self.Internals['LastPwr'] < 100:
                    self.switchHeat(False)
                # if power was 100 (i.e. a full cycle), then we let the next calculation (at next heartbeat) decide
                # to switch off in order to avoid potentially damaging quick off/on cycles to the heater(s)

            elif self.pause and not self.pauserequested:  # we are in pause and the pause switch is now off
                if self.pauserequestchangedtime + timedelta(minutes=self.pauseoffdelay) <= now:
                    self.WriteLog("Pause is now Off", "Status")
                    self.pause = False
                    self.nextcalc = now  # this will force a recalculation on the next heartbeat

            elif not self.pause and self.pauserequested:  # we are not in pause and the pause switch is now on
                if self.pauserequestchangedtime + timedelta(minutes=self.pauseondelay) <= now:
                    self.WriteLog("Pause is now On", "Status")
                    self.pause = True
                    self.heat = False  # [FIX] explicitly reset heat flag in case switchHeat() returned early
                    self.switchHeat(False)
                    # [FIX] reset valve wait state when pause is activated
                    self.heatduration_pending = 0
                    self.valveopen = False
                    self.valveWaitStart = None

            elif self.valveidx > 0 and self.heatduration_pending > 0 and not self.valveopen:
                # [NEW] valve sensor configured: we commanded heating but are waiting for valve to open
                if self.isValveOpen():
                    self.valveopen = True
                    self.endheat = now + timedelta(minutes=self.heatduration_pending)
                    self.heatduration_pending = 0
                    self.WriteLog("Valve is now open - heat timer started, end heat = {}".format(
                        self.endheat), "Status")
                else:
                    waited = round((now - self.valveWaitStart).total_seconds() / 60, 1) if self.valveWaitStart else 0
                    self.WriteLog("Waiting for valve to open... ({} min elapsed)".format(waited), "Verbose")

            elif (self.nextcalc <= now) and not self.pause:  # we start a new calculation
                self.nextcalc = now + timedelta(minutes=self.calculate_period)
                self.WriteLog("Next calculation time will be : " + str(self.nextcalc), "Verbose")

                # make current setpoint used in calculation reflect the selected mode (10=normal, 20=economy)
                if Devices[2].sValue == "10":
                    self.setpoint = float(Devices[4].sValue)
                else:
                    self.setpoint = float(Devices[5].sValue)

                # [NEW] update beta from external temperature history every 24h
                self.updateBeta()

                # call the Domoticz json API for a temperature devices update, to get the latest temps...
                if self.readTemps():
                    # do the thermostat work
                    self.AutoMode()
                else:
                    # make sure we switch off heating if there was an error with reading the temp
                    self.switchHeat(False)

        if self.nexttemps <= now:
            # call the Domoticz json API for a temperature devices update, to get the latest temps (and avoid the
            # connection time out after 10 mins that floods domoticz logs in versions of domoticz since spring 2018)
            self.readTemps()

        # check if need to refresh setpoints so that they do not turn red in GUI
        if self.nextupdate <= now:
            self.nextupdate = now + timedelta(minutes=int(Settings["SensorTimeout"]))
            Devices[4].Update(nValue=0, sValue=Devices[4].sValue)
            Devices[5].Update(nValue=0, sValue=Devices[5].sValue)


    def AutoMode(self):

        self.WriteLog("Temperatures: Inside = {} / Outside = {}".format(self.intemp, self.outtemp), "Verbose")

        if self.intemp > self.setpoint + self.deltamax:
            self.WriteLog("Temperature exceeds setpoint", "Verbose")
            overshoot = True
            power = 0
            # [IMPROVED] reset integral on overshoot to avoid windup fighting against natural cooldown
            self.integral_error = 0.0
        else:
            overshoot = False
            if self.learn:
                self.AutoCallib()
            else:
                self.learn = True
            if self.outtemp is None:
                power = round((self.setpoint - self.intemp) * self.Internals["ConstC"], 1)
            else:
                power = round((self.setpoint - self.intemp) * self.Internals["ConstC"] +
                              (self.setpoint - self.outtemp) * self.Internals["ConstT"], 1)

            # [NEW] PI controller: add integral term to correct steady-state drift below setpoint.
            # The integral accumulates the temperature error each cycle and contributes a gentle
            # correction to power. Anti-windup clamp prevents unbounded accumulation.
            current_error = self.setpoint - self.intemp
            self.integral_error = max(-self.integral_max / self.Ki,
                                      min(self.integral_max / self.Ki,
                                          self.integral_error + current_error))
            integral_contribution = round(self.Ki * self.integral_error, 1)
            self.WriteLog("PI: error={}, integral={}, contribution={}".format(
                round(current_error, 2), round(self.integral_error, 2), integral_contribution), "Verbose")
            power = round(power + integral_contribution, 1)

        if power < 0:
            power = 0  # lower limit
        elif power > 100:
            power = 100  # upper limit

        # apply minimum power as required
        if power <= self.minheatpower and (Parameters["Mode4"] == "Forced" or not overshoot):
            self.WriteLog(
                "Calculated power is {}, applying minimum power of {}".format(power, self.minheatpower), "Verbose")
            power = self.minheatpower

        # [IMPROVED] Boost mode: instead of jumping straight to 100%, apply a proportional ramp
        # between boostgap and 2*boostgap, capped at boostmaxpower (85% by default).
        # This avoids hard overshoot while still reacting quickly when far below setpoint.
        if self.boost:
            gap = self.setpoint - self.intemp
            if gap > self.boostgap:
                ramp_factor = min(1.0, (gap - self.boostgap) / self.boostgap)
                boost_power = round(power + ramp_factor * (self.boostmaxpower - power), 1)
                if boost_power > power:
                    self.WriteLog(
                        "Boost mode: gap={}°C, ramp={:.0f}%, power {} -> {}".format(
                            round(gap, 2), ramp_factor * 100, power, boost_power), "Verbose")
                    power = boost_power

        if power > 100:
            power = 100  # final upper limit after boost

        heatduration = round(power * self.calculate_period / 100)
        self.WriteLog("Calculation: Power = {} -> heat duration = {} minutes".format(power, heatduration), "Verbose")

        if power == 0:
            self.switchHeat(False)
            self.heatduration_pending = 0
            self.valveopen = False
            self.valveWaitStart = None
            self.WriteLog("No heating requested !", "Verbose")
        else:
            if self.valveidx > 0:
                # valve sensor configured: command the heater switch (opens valve),
                # but store heat duration as pending - timer starts only when valve contact confirms open
                if self.isValveOpen():
                    # valve already open (e.g. other zone already opened it)
                    self.valveopen = True
                    self.heatduration_pending = 0
                    self.endheat = datetime.now() + timedelta(minutes=heatduration)
                    self.WriteLog("Valve already open - heat timer started immediately, end heat = {}".format(
                        self.endheat), "Status")
                else:
                    self.valveopen = False
                    self.heatduration_pending = heatduration
                    self.valveWaitStart = datetime.now()
                    # set endheat far in the future so the cycle does not expire while waiting
                    self.endheat = datetime.now() + timedelta(minutes=self.calculate_period + 20)
                    self.WriteLog("Valve not yet open - waiting for contact before starting heat timer ({} min pending)".format(
                        heatduration), "Status")
            else:
                # no valve sensor: original behaviour
                self.endheat = datetime.now() + timedelta(minutes=heatduration)
            self.WriteLog("End Heat time = " + str(self.endheat), "Verbose")
            self.switchHeat(True)
            self.Internals['LastPwr'] = power
            self.Internals['LastInT'] = self.intemp
            self.Internals['LastOutT'] = self.outtemp
            self.Internals['LastSetPoint'] = self.setpoint
            if self.Internals["ALStatus"] != 2:
                self.Internals['ALStatus'] = 1
                self.saveUserVar()  # update user variables with latest learning

        self.lastcalc = datetime.now()


    def AutoCallib(self):

        now = datetime.now()
        # [FIX] skip calibration if lastcalc is None (first cycle) or zero timedelta to avoid division anomalies
        if self.lastcalc is None or (now - self.lastcalc).total_seconds() < 1:
            self.WriteLog("AutoCallib: skipping - lastcalc not yet set or zero timedelta", "Verbose")
            return
        if self.Internals['ALStatus'] != 1:  # not initialized... do nothing
            self.WriteLog("First pass at AutoCallib... no calibration", "Verbose")
            pass
        elif self.Internals['LastPwr'] == 0:  # heater was off last time, do nothing
            self.WriteLog("Last power was zero... no calibration", "Verbose")
            pass
        elif self.Internals['LastPwr'] == 100 and self.intemp < self.Internals['LastSetPoint']:
            # heater was on max and setpoint was NOT reached: no learning, but
            # [FIX] if ConstC is already very high, it means it is diverging - force it down gradually
            if self.Internals['ConstC'] > 150:
                self.Internals['ConstC'] = round(self.Internals['ConstC'] * 0.85, 1)
                self.WriteLog("ConstC too high and power was 100% but setpoint not reached - forcing down to {}".format(
                    self.Internals['ConstC']), "Status")
            else:
                self.WriteLog("Last power was 100% but setpoint not reached... no calibration", "Verbose")
        elif self.Internals['ConstC'] > 120 and self.intemp >= self.Internals['LastSetPoint']:
            # [FIX] ConstC is approaching maximum and setpoint was reached: ConstC is too high for current
            # conditions (e.g. spring/autumn). Force it down gradually at 15% per cycle.
            # This is symmetric with the ConstT recovery logic and does not prevent ConstC from rising
            # again in winter if needed (EMA learning will raise it when setpoint is not reached).
            self.Internals['ConstC'] = round(self.Internals['ConstC'] * 0.85, 1)
            self.WriteLog("ConstC above 120 and setpoint reached - forcing down to {}".format(
                self.Internals['ConstC']), "Status")
        elif self.Internals['LastPwr'] == 100 and self.intemp >= self.Internals['LastSetPoint']:
            # [FIX] heater was on max AND setpoint was reached/exceeded: ConstC is too high, learn downward
            alpha_C = max(1.0 / (self.Internals['nbCC'] + 1), 0.02)
            ConstC_new = (self.Internals['ConstC'] * ((self.Internals['LastSetPoint'] - self.Internals['LastInT']) /
                                                       max(self.intemp - self.Internals['LastInT'], 0.1) *
                                                       (timedelta.total_seconds(now - self.lastcalc) /
                                                        (self.calculate_period * 60))))
            self.WriteLog("LastPwr was 100% but setpoint reached - learning ConstC down: new calc = {}".format(
                ConstC_new), "Verbose")
            self.Internals['ConstC'] = round(
                (1 - alpha_C) * self.Internals['ConstC'] + alpha_C * ConstC_new, 1)
            self.Internals['nbCC'] = min(self.Internals['nbCC'] + 1, 50)
            self.WriteLog("ConstC updated to {} (alpha={})".format(
                self.Internals['ConstC'], round(alpha_C, 3)), "Verbose")
        elif self.intemp > self.Internals['LastInT'] and self.Internals['LastSetPoint'] > self.Internals['LastInT']:
            # [IMPROVED] learning ConstC via Exponential Moving Average (EMA) instead of simple capped average.
            # EMA reacts faster to physical changes (new heater, renovations, etc.)
            # alpha converges from 1.0 (first sample) down to 0.02 (minimum, keeps adapting gently forever)
            alpha_C = max(1.0 / (self.Internals['nbCC'] + 1), 0.02)
            ConstC_new = (self.Internals['ConstC'] * ((self.Internals['LastSetPoint'] - self.Internals['LastInT']) /
                                                       (self.intemp - self.Internals['LastInT']) *
                                                       (timedelta.total_seconds(now - self.lastcalc) /
                                                        (self.calculate_period * 60))))
            self.WriteLog("New calc for ConstC = {}".format(ConstC_new), "Verbose")
            self.Internals['ConstC'] = round(
                (1 - alpha_C) * self.Internals['ConstC'] + alpha_C * ConstC_new, 1)
            self.Internals['nbCC'] = min(self.Internals['nbCC'] + 1, 50)
            self.WriteLog("ConstC updated to {} (alpha={})".format(
                self.Internals['ConstC'], round(alpha_C, 3)), "Verbose")

        elif (self.outtemp is not None and self.Internals['LastOutT'] is not None) and \
                self.Internals['LastSetPoint'] > self.Internals['LastOutT']:
            # [FIX] learning ConstT via EMA - same approach as ConstC.
            # Previous formula used += delta which could only grow over time and never converge downward.
            # EMA allows ConstT to decrease naturally when outside temperatures rise (e.g. spring/summer).
            # [NEW] beta reduces effective_nbCT proportionally to recent external temperature variability:
            # higher beta (large daily ranges) -> lower effective_nbCT -> higher alpha_T -> faster adaptation.
            beta = self.Internals.get('beta', 0.0)
            effective_nbCT = max(0, self.Internals['nbCT'] - int(beta))
            alpha_T = max(1.0 / (effective_nbCT + 1), 0.02)
            ConstT_new = ((self.Internals['LastSetPoint'] - self.intemp) /
                          max(self.Internals['LastSetPoint'] - self.Internals['LastOutT'], 0.1) *
                          self.Internals['ConstC'] *
                          (timedelta.total_seconds(now - self.lastcalc) /
                           (self.calculate_period * 60)))
            self.WriteLog("New calc for ConstT = {} (beta={}, effective_nbCT={}, alpha={})".format(
                round(ConstT_new, 3), beta, effective_nbCT, round(alpha_T, 3)), "Verbose")
            self.Internals['ConstT'] = round(
                (1 - alpha_T) * self.Internals['ConstT'] + alpha_T * ConstT_new, 1)
            self.Internals['nbCT'] = min(self.Internals['nbCT'] + 1, 50)
            self.WriteLog("ConstT updated to {} (alpha={})".format(
                self.Internals['ConstT'], round(alpha_T, 3)), "Verbose")

        # [FIX] if ConstT is at maximum and setpoint was reached, ConstT is too high.
        # Force it down gradually at 15% per cycle, same recovery logic as ConstC divergence.
        if self.Internals['ConstT'] >= 10.0 and self.intemp >= self.Internals['LastSetPoint']:
            self.Internals['ConstT'] = round(self.Internals['ConstT'] * 0.85, 1)
            self.WriteLog("ConstT at maximum and setpoint reached - forcing down to {}".format(
                self.Internals['ConstT']), "Status")

        # [NEW] Safety clamp: ConstC and ConstT must stay within reasonable bounds in both directions.
        # Upper bounds: ConstC > 150 means even 0.5°C error gives 75% power - clearly wrong.
        #               ConstT > 10 is unrealistic for any normal installation.
        # Lower bounds: ConstC < 1.0 means even 10°C error gives less than 1% power - also wrong.
        #               ConstT < 0.0 is physically meaningless (negative external contribution).
        if self.Internals['ConstC'] > 150:
            self.Internals['ConstC'] = 150.0
            self.WriteLog("ConstC clamped to maximum of 150", "Status")
        elif self.Internals['ConstC'] < 1.0:
            self.Internals['ConstC'] = 1.0
            self.WriteLog("ConstC clamped to minimum of 1.0", "Status")

        if self.Internals['ConstT'] > 10:
            self.Internals['ConstT'] = 10.0
            self.WriteLog("ConstT clamped to maximum of 10", "Status")
        elif self.Internals['ConstT'] < 0.0:
            self.Internals['ConstT'] = 0.0
            self.WriteLog("ConstT clamped to minimum of 0.0", "Status")



    def updateBeta(self):
        """Fetch last 7 days of external temperature history and compute beta
        as the average daily range (max - min). Called every 24h.
        If no external sensor is configured, beta remains 0.0 (no effect on learning).
        """
        if not self.OutTempSensors:
            return  # no external sensor configured, nothing to do

        today = datetime.now().strftime("%Y-%m-%d")
        if self.Internals.get('LastBetaUpdate', '') == today:
            return  # already updated today

        idx = self.OutTempSensors[0]  # use first external sensor
        apiresult = DomoticzAPI("type=command&param=graph&sensor=temp&idx={}&range=month".format(idx))
        if not apiresult or "result" not in apiresult:
            Domoticz.Error("updateBeta: failed to fetch temperature history for idx={}".format(idx))
            return

        # take last 7 complete days (skip today which may be incomplete at time of calculation)
        complete_data = [d for d in apiresult["result"] if d["d"] != today]
        data = complete_data[-7:]
        if len(data) == 0:
            return

        ranges = []
        for day in data:
            try:
                te = float(day["te"])
                tm = float(day["tm"])
                day_range = round(te - tm, 1)
                ranges.append(day_range)
                self.WriteLog("Beta day {}: max={}, min={}, range={}°C".format(
                    day["d"], te, tm, day_range), "Verbose")
            except Exception:
                pass

        if ranges:
            beta = round(sum(ranges) / len(ranges), 1)
            self.Internals['beta'] = beta
            self.Internals['LastBetaUpdate'] = today
            self.saveUserVar()
            self.WriteLog("Beta updated: avg daily range over last {} days = {}°C".format(
                len(ranges), beta), "Status")

    def switchHeat(self, switch):

        # Build list of heater switches, with their current status,
        # to be used to check if any of the heaters is already in desired state
        switches = {}
        devicesAPI = DomoticzAPI("type=command&param=getdevices&filter=light&used=true&order=Name")
        if devicesAPI:
            for device in devicesAPI["result"]:  # parse the switch device
                idx = int(device["idx"])
                if idx in self.Heaters:  # this switch is one of our heaters
                    if "Status" in device:
                        switches[idx] = True if device["Status"] == "On" else False
                        self.WriteLog("Heater switch {} currently is '{}'".format(idx, device["Status"]), "Verbose")
                    else:
                        Domoticz.Error("Device with idx={} does not seem to be a switch !".format(idx))

        # fool proof checking.... based on users feedback
        if len(switches) == 0:
            Domoticz.Error("none of the devices in the 'heaters' parameter is a switch... no action !")
            return

        # flip on / off as needed
        self.heat = switch
        command = "On" if switch else "Off"
        self.WriteLog("Heating '{}'".format(command), "Verbose")
        for idx in self.Heaters:
            if switches[idx] != switch:  # check if action needed
                DomoticzAPI("type=command&param=switchlight&idx={}&switchcmd={}".format(idx, command))
        if switch:
            self.WriteLog("End Heat time = " + str(self.endheat), "Verbose")


    def isValveOpen(self):
        """Read the valve contact sensor status via Domoticz API. Returns True if On (valve open)."""
        # [FIX] use rid= to fetch only the specific device instead of the full list
        devicesAPI = DomoticzAPI("type=command&param=getdevices&rid={}".format(self.valveidx))
        if devicesAPI and "result" in devicesAPI and len(devicesAPI["result"]) > 0:
            device = devicesAPI["result"][0]
            status = device.get("Status", "Off")
            self.WriteLog("Valve contact sensor idx={} status={}".format(self.valveidx, status), "Verbose")
            return status == "On"
        Domoticz.Error("Valve contact sensor idx={} not found !".format(self.valveidx))
        return False


    def readTemps(self):

        # set update flag for next temp update
        self.nexttemps = datetime.now() + timedelta(minutes=5)

        # fetch all the devices from the API and scan for sensors
        noerror = True
        listintemps = []
        listouttemps = []
        devicesAPI = DomoticzAPI("type=command&param=getdevices&filter=temp&used=true&order=Name")
        if devicesAPI:
            for device in devicesAPI["result"]:  # parse the devices for temperature sensors
                idx = int(device["idx"])
                if idx in self.InTempSensors:
                    if "Temp" in device:
                        self.WriteLog("device: {}-{} = {}".format(device["idx"], device["Name"], device["Temp"]), "Verbose")
                        # check temp sensor is not timed out
                        if not self.SensorTimedOut(idx, device["Name"], device["LastUpdate"]):
                            listintemps.append(device["Temp"])
                    else:
                        Domoticz.Error("device: {}-{} is not a Temperature sensor".format(device["idx"], device["Name"]))
                elif idx in self.OutTempSensors:
                    if "Temp" in device:
                        self.WriteLog("device: {}-{} = {}".format(device["idx"], device["Name"], device["Temp"]), "Verbose")
                        # check temp sensor is not timed out
                        if not self.SensorTimedOut(idx, device["Name"], device["LastUpdate"]):
                            listouttemps.append(device["Temp"])
                    else:
                        Domoticz.Error("device: {}-{} is not a Temperature sensor".format(device["idx"], device["Name"]))

        # calculate the average inside temperature
        nbtemps = len(listintemps)
        if nbtemps > 0:
            self.intemp = round(sum(listintemps) / nbtemps, 1)
            # update the dummy device showing the current thermostat temp
            Devices[6].Update(nValue=0, sValue=str(self.intemp), TimedOut=False)
            if self.intemperror:  # there was previously an invalid inside temperature reading... reset to normal
                self.intemperror = False
                self.WriteLog("Inside Temperature reading is now valid again: Resuming normal operation", "Status")
                # we remove the timedout flag on the thermostat switch
                Devices[1].Update(nValue=Devices[1].nValue, sValue=Devices[1].sValue, TimedOut=False)
        else:
            # no valid inside temperature
            noerror = False
            if not self.intemperror:
                self.intemperror = True
                Domoticz.Error("No Inside Temperature found: Switching heating Off")
                self.switchHeat(False)
                # we mark both the thermostat switch and the thermostat temp devices as timedout
                Devices[1].Update(nValue=Devices[1].nValue, sValue=Devices[1].sValue, TimedOut=True)
                Devices[6].Update(nValue=Devices[6].nValue, sValue=Devices[6].sValue, TimedOut=True)

        # calculate the average outside temperature
        nbtemps = len(listouttemps)
        if nbtemps > 0:
            self.outtemp = round(sum(listouttemps) / nbtemps, 1)
        else:
            self.WriteLog("No Outside Temperature found...", "Verbose")
            self.outtemp = None

        self.WriteLog("Inside Temperature = {}".format(self.intemp), "Verbose")
        self.WriteLog("Outside Temperature = {}".format(self.outtemp), "Verbose")
        return noerror


    def getUserVar(self):

        variables = DomoticzAPI("type=command&param=getuservariables")
        if variables:
            # there is a valid response from the API but we do not know if our variable exists yet
            novar = True
            varname = Parameters["Name"] + "-InternalVariables"
            valuestring = ""
            if "result" in variables:
                for variable in variables["result"]:
                    if variable["Name"] == varname:
                        valuestring = variable["Value"]
                        novar = False
                        break
            if novar:
                # create user variable since it does not exist
                self.WriteLog("User Variable {} does not exist. Creating it.".format(varname), "Verbose")
                DomoticzAPI("type=command&param=adduservariable&vname={}&vtype=2&vvalue={}".format(
                    varname, parse.quote(json.dumps(self.InternalsDefaults))))
                self.Internals = self.InternalsDefaults.copy()
            else:
                # [IMPROVED] use json.loads() instead of eval() for safe deserialization.
                # On first run after upgrade from older version, the variable is in Python dict format:
                # recover it via ast.literal_eval (safe subset of eval) and immediately re-save as JSON.
                try:
                    self.Internals.update(json.loads(valuestring))
                except Exception:
                    try:
                        import ast
                        self.Internals.update(ast.literal_eval(valuestring))
                        self.WriteLog("Internal variables loaded via legacy format. Re-saving as JSON.", "Status")
                        self.saveUserVar()  # immediately convert to JSON format
                    except Exception:
                        Domoticz.Error("Cannot parse internal variables, resetting to defaults")
                        self.Internals = self.InternalsDefaults.copy()
                return
        else:
            Domoticz.Error("Cannot read the uservariable holding the persistent variables")
            self.Internals = self.InternalsDefaults.copy()


    def saveUserVar(self):

        # [IMPROVED] save as JSON string instead of Python repr string (safer, standard)
        varname = Parameters["Name"] + "-InternalVariables"
        DomoticzAPI("type=command&param=updateuservariable&vname={}&vtype=2&vvalue={}".format(
            varname, parse.quote(json.dumps(self.Internals))))


    def WriteLog(self, message, level="Normal"):

        if (self.loglevel == "Verbose" and level == "Verbose") or level == "Status":
            Domoticz.Status(message)
        elif level == "Normal":
            Domoticz.Log(message)


    def SensorTimedOut(self, idx, name, datestring):

        def LastUpdate(datestring):
            dateformat = "%Y-%m-%d %H:%M:%S"
            # the below try/except is meant to address an intermittent python bug in some embedded systems
            try:
                result = datetime.strptime(datestring, dateformat)
            except TypeError:
                result = datetime(*(time.strptime(datestring, dateformat)[0:6]))
            return result

        timedout = LastUpdate(datestring) + timedelta(minutes=int(Settings["SensorTimeout"])) < datetime.now()

        # handle logging of time outs... only log when status changes (less clutter in logs)
        if timedout:
            if self.ActiveSensors[idx]:
                Domoticz.Error("skipping timed out temperature sensor '{}'".format(name))
                self.ActiveSensors[idx] = False
        else:
            if not self.ActiveSensors[idx]:
                self.WriteLog("previously timed out temperature sensor '{}' is back online".format(name), "Status")
                self.ActiveSensors[idx] = True

        return timedout


global _plugin
_plugin = BasePlugin()


def onStart():
    global _plugin
    _plugin.onStart()


def onStop():
    global _plugin
    _plugin.onStop()


def onCommand(Unit, Command, Level, Color):
    global _plugin
    _plugin.onCommand(Unit, Command, Level, Color)


def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()


# Plugin utility functions ---------------------------------------------------

def parseCSV(strCSV):
    """Parse a comma-separated string of integers. Non-integer values are silently skipped."""
    listvals = []
    for value in strCSV.split(","):
        try:
            listvals.append(int(value))
        except ValueError:
            pass
    return listvals


def DomoticzAPI(APICall):

    resultJson = None
    url = "http://{}:{}/json.htm?{}".format(Parameters["Address"], Parameters["Port"], parse.quote(APICall, safe="&="))
    Domoticz.Debug("Calling domoticz API: {}".format(url))
    try:
        req = request.Request(url)
        if Parameters["Username"] != "":
            Domoticz.Debug("Add authentification for user {}".format(Parameters["Username"]))
            credentials = ('%s:%s' % (Parameters["Username"], Parameters["Password"]))
            encoded_credentials = base64.b64encode(credentials.encode('ascii'))
            req.add_header('Authorization', 'Basic %s' % encoded_credentials.decode("ascii"))

        response = request.urlopen(req)
        if response.status == 200:
            resultJson = json.loads(response.read().decode('utf-8'))
            if resultJson["status"] != "OK":
                Domoticz.Error("Domoticz API returned an error: status = {}".format(resultJson["status"]))
                resultJson = None
        else:
            Domoticz.Error("Domoticz API: http error = {}".format(response.status))
    except Exception as e:
        Domoticz.Error("Error calling '{}': {}".format(url, str(e)))
    return resultJson


def CheckParam(name, value, default):
    if type(default) is int and type(value) is int:
        param = value
    elif type(default) is float and type(value) is float:
        param = value
    else:
        param = default
        Domoticz.Error("Parameter '{}' has an invalid value of '{}' ! default of '{}' is instead used.".format(name, value, default))
    return param


# Generic helper functions
def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug("'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))
    return