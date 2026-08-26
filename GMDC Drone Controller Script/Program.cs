using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Data.Common;
using System.Drawing.Imaging;
using System.Linq;
using System.Net;
using System.Runtime.CompilerServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {


        #region mdk preserve
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            manageFirstLoad(Storage, Me.CustomData);
            firstload = true;
        }
        //default information
        string drone_tag = "SWRM_D"; //Mining drone group tag
        double drone_length = 2.6;
        double drone_clear_offset = 12.0; //drill clear mode distance offset
        string secondary = ""; //vessel/rig name (optional)

        //display surface indexes
        int srfM = 0;
        int srfL = 0;
        int srfD = 0;
        int srfV = 0;
        int drones_per_screen = 8;
        int droneUndockDelayTime = 6;
        int undock_delay_limit = 12;

        //Drone Comms
        int droneCommunicationsProcessingDelay = 0;
        int droneCommunicationsPingDelay = 30;

        #endregion

        #region static_variables
        //visualiser settings
        int spriteCountLimit = 500;
        int spritecount_limit_insert = 250;
        //statics
        int game_factor = 10;
        string ver = "V0.615B";
        string comms = "Comms";
        string MainS = "Main";
        string DroneS = "Drone";
        string IntfS = "Interface";
        string LstS = "List";
        string dspy = "Display";
        string GrphS = "Visual";
        bool coreOutGrid = false;
        int clbs = 44;
        double bclu = 30.0;
        string tx_chan;
        bool dronesLaunchedStatus = false;
        int dronesInFlightFactor = 1;
        int dronesActiveHardLimit = 10;
        int skipBoresNumber = 0;
        double gridSize;
        int numPointsY;
        int numPointsX;
        bool miningCoordsValid = false;
        bool gridCreated = false;
        int maxActiveDronesCount;
        string droneDataInput;
        string prospectorDataInput;
        double bclm = 1.0;
        string receivedDroneName;
        string receivedDroneDamageStatus;
        string receivedDroneTunnelFinished;
        string receivedDroneStatus;
        string receivedDroneDocked;
        string receivedDroneUndocked;
        string recived_drone_autopilot;
        string rc_dn_drl_dpth;
        string rc_dn_drl_crnt;
        string rc_dn_drl_strt;
        string rc_dn_gps_lst;
        string rc_dn_cargo_full;
        string rc_dn_rchg_req;
        string recievedDroneAutdock;
        string recievedDroneDockingReady;
        string rc_auto_pilot_enabled;
        int recieved_drone_list_position;
        double rc_d_cn = 0.0;
        string rc_locx;
        string rc_locy;
        string rc_locz;
        string rc_dn_chg;
        string rc_dn_gas;
        string rc_dn_str;
        int currentGPSIndex = 0;
        int realGPSIndex = 0;
        string rxChannelDrone = "";
        string rxChannelProspector = "";
        string tx_recall_channel = "";
        string droneTXRecallChannel = "";
        string txDronePingChannel = "";
        string txDroneSyncChannel = "";
        string pingMessage = "ping";
        string syncMessage = "";
        bool dronesPinged = false;
        string antennaTagName = "";
        string lightsTagName = "";
        string dp_mn_tag = "";
        string dp_drn_tag = "";
        string dp_lst_tag = "";
        string dp_vis_tag = "";
        string interfaceTag = "";
        double drillLength;
        bool mainCustomDataValid = false;
        bool canRun = false;
        bool canReset = false;
        bool canTransmit = false;
        bool mustRecall_Command = false;
        bool mustFreeze_Command = false;
        bool canInit = false;
        //bool found = false;
        bool generalReset;
        bool miningGridValid = false;

        double ignoreDepth = 0.0;
        double safe_dstvl = 0.0;
        bool prospectTargetValid = false;
        bool prospectAlignTargetValid = false;
        bool customDataAlignTargetValid = false;
        string commandAsk;
        //string customData1;
        string customData2;
        string customData3;
        string customData4;
        string customData5;
        string customData6;
        string customData7;
        string customData8;
        string customData9;
        string customData10;
        string customData11;
        string customData12;
        string customData13;
        string customData14;
        string customData15;
        //string customData16;
        // string customData17;
        string customData18;
        string customData19;
        string customData20;
        //  string customData21;
        string customData22;
        //string customData23;
        //   string remoteControlCustomData1 = "";
        string remoteControlCustomData2 = "";
        string remoteControlCustomData3 = "";
        string remoteControlCustomData4 = "";
        //   string remoteControlCustomData5 = "";
        string remoteControlCustomData6 = "";
        //  string remoteControlCustomData7 = "";
        //  string remoteControlCustomData8 = "";
        string remoteControlCustomData9 = "";
        string remoteControlCustomData10 = "";
        string remoteControlCustomData11 = "";
        //  string remoteControlCustomData12 = "";

        string cd1 = "";
        string xp = "";
        string yp = "";
        string zp = "";
        string cd5 = "";
        string cm = "";
        string cd6 = "";
        string igd = "";
        string xp2 = "";
        string yp2 = "";
        string zp2 = "";
        int totalMiningSequencesComplete = 0;
        int totalMiningRuns = 1;
        int totalDronesActive = 0;
        int totalDronesMining = 0;
        int t_drn_dckg = 0;
        int t_drn_dck = 0;
        int t_drn_udckg = 0;
        int t_drn_udck = 0;
        int t_drn_rechg = 0;
        int t_drn_unload = 0;
        int t_drn_idle_undocked = 0;
        int t_drn_idle_docked = 0;
        int t_drn_exit = 0;
        int t_drn_mine = 0;
        int t_drn_nav = 0;
        int boresRemaining;
        bool faultLightOutput = false;
        int faultCounter = 0;
        private IEnumerator<bool> gridCoroutine;
        private IEnumerator<bool> listCoroutine;
        private IEnumerator<bool> visCoroutine;
        IMyRadioAntenna antennaActual;
        IMyLightingBlock lightIndicatorActual;
        IMyRemoteControl remoteControlActual;
        IMyProgrammableBlock pbInterfaceActual;
        Vector3D miningGPSCoordinates;
        Vector3D gridCentreGPSCoordinates;
        //  Vector3D next_gps_crds;
        Vector3D targetGPSCoordinates;
        Vector3D alignGPSCoordinates;
        Vector3D planeNrml;
        StringBuilder miningCoordinatesNew = new StringBuilder();
        StringBuilder c = new StringBuilder();
        List<Vector3D> gridBorePosition = new List<Vector3D>();
        List<bool> gridBoreOccupied = new List<bool>();
        List<bool> gridBoreFinished = new List<bool>();
        List<string> cl = new List<string>();
        List<string> cl2 = new List<string>();
        //int cbval = 0;
        //bool clbt = false;
        int gridBoresCompleted;
        int gpsGridPositionValue = -1;
        string drone_namer = "";
        StringBuilder droneInformation = new StringBuilder();
        StringBuilder displayTextMain = new StringBuilder();
        StringBuilder displayTextList = new StringBuilder();
        StringBuilder jxt = new StringBuilder();
        List<bool> droneMining = new List<bool>();
        bool setupComplete = false;
        bool timeDelayed = false;
        int timeCounter = 0;
        int dronePingTimerCount = 0;
        bool readyFlag = false;
        int droneResetStatusCount = 0;
        int droneDockedStatusCount = 0;
        string screenStatus = "Idle";
        string replyC = "reply";
        string prospC = "prospector";
        string syncC = "sync";
        string commandRecall = "recall";
        string commandOperate = "operate";
        bool mustUndockCommand = false;
        bool disableRunArgument = false;
        bool renew_header = true;
        Color Cgreen = new Color(0, 255, 0);
        Color Cyellow = new Color(255, 255, 0);
        Color Cred = new Color(255, 0, 0);
        Color Cblue = new Color(0, 0, 255);
        Color Coren = new Color(235, 90, 33);
        bool canInterfaceCommand = false;
        bool i_init = false;
        bool i_res = false;
        bool i_run = false;
        bool i_recall = false;
        bool i_eject = false;
        bool i_frz = false;
        bool i_stop = false;
        bool noInterfaceCommand = false;
        string interfaceArgument;
        //decimal dps_r_d = 0.0m;
        List<IMyRemoteControl> remoteControlAll = new List<IMyRemoteControl>();
        List<IMyRemoteControl> remoteControlTag = new List<IMyRemoteControl>();
        List<IMyRadioAntenna> antennaAll = new List<IMyRadioAntenna>();
        List<IMyRadioAntenna> antennaTag = new List<IMyRadioAntenna>();
        List<IMyLightingBlock> lightsAll = new List<IMyLightingBlock>();
        List<IMyLightingBlock> lightsTag = new List<IMyLightingBlock>();
        List<IMyTerminalBlock> display_all = new List<IMyTerminalBlock>();
        List<IMyTerminalBlock> display_tag_main = new List<IMyTerminalBlock>();
        List<IMyTerminalBlock> display_tag_list = new List<IMyTerminalBlock>();
        List<IMyTerminalBlock> display_tag_drone = new List<IMyTerminalBlock>();
        List<IMyTerminalBlock> display_tag_vis = new List<IMyTerminalBlock>();
        List<IMyProgrammableBlock> programblockAll = new List<IMyProgrammableBlock>();
        List<IMyProgrammableBlock> interfacePBTag = new List<IMyProgrammableBlock>();
        List<IMyDoor> _allDoorsCache = new List<IMyDoor>();
        IMyTextSurface sD;
        IMyTextSurface sM;
        IMyTextSurface sL;
        IMyTextSurface sV;
        IMyCubeGrid meCubeGrid;
        RectangleF _viewport;
        StringBuilder sb = new StringBuilder();
        int totalDronesDamaged = 0;
        int totalDronesUnknown = 0;
        //  int t_dn_ok = 0;
        string g1;
        string g2;
        //int di = 0;
        int total_drones_undocking = 0;
        int undockTimer = 0;
        bool dronesUndocking = false;
        bool canLoading = false;
        string px = "";
        string py = "";
        string pz = "";
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        int initialisedGridCount = 0;
        bool gridInitialisationComplete = false;
        //int debugcount = 0;
        bool bores_regen;
        bool listGeneratorFinished = false;
        bool listHeaderGenerated = false;
        bool frame_generator_finished = false;
        double percent_list = 0.0;
        double percent_list_vis = 0.0;
        double percent_list_drones = 0.0;
        double percent_grid = 0.0;
        string icon = "";
        int stateshift = 0;
        //string temp_id_name;
        //string temp_id_name_2;
        string secondary_tag = "";
        double game_tick_length = 16.666;

        IMyBroadcastListener listenDrones;
        IMyBroadcastListener listenProspector;
        List<MyIGCMessage> droneMessagesBuffer = new List<MyIGCMessage>();
        List<MyIGCMessage> prospectorMessagesBuffer = new List<MyIGCMessage>();
        bool prospectorMessageReceived = false;
        bool droneMessageConfirmed = false;
        int receivedDroneNameIndex = -1;
        bool droneMessageReceived = false;
        bool Visport_OK = false;
        List<MySprite> sprites = new List<MySprite>();
        int spriteCounter = 0;
        bool spriteInsert = false;
        StringBuilder customDataString = new StringBuilder();
        string _oldCustomData = "";
        StringBuilder textSpriteBuffer = new StringBuilder();

        private double totalRuntimeMs = 0.0;
        private int runCount = 0;

        //  private double averageRuntimeMs = 0.0;

        MyIni _ini = new MyIni();
        MyIni _antennaStore = new MyIni();
        string runargument = "";
        bool firstload = false;
        MyIni _customDataStore = new MyIni();
        MyIni _remoteDataStore = new MyIni();
        // MyIni _commsData = new MyIni();
        string jobdata = "";
        string rcjobdata = "";
        string jobinfo = "Jobinfo";
        string gmdccategory = "GMDCJobData";
        //   string gmdpcategory = "GMDPJobData";
        string rcjobinfo = "Jobinfo";
        //   string prospectmain = "maindata";
        //   string prospecttarget = "aligndata";
        string _cachedCustomData = "";
        int _frameCounter = 0;
        StringBuilder sbtexttemp = new StringBuilder();
        MyIni safebuilder = new MyIni();
        MyIni _interfaceCommand = new MyIni();
        bool loadsave = false;
        string jobname = "Default";
        string rcdataOld = "";
        string interfacecommandOld = "";
        string incomingName = "";
        List<string> droneID = new List<string>();
        Dictionary<string, DroneData> Swarm = new Dictionary<string, DroneData>();
        bool perimeterOnly = false;
        int perimiterInt = 0;
        bool rotateHome = true;
        float scale1;
        string datatemp = "";

        List<IMyMotorStator> rotors_all = new List<IMyMotorStator>();
        List<IMyMotorAdvancedStator> rotorAdvancedStators_all = new List<IMyMotorAdvancedStator>();
        List<IMyPistonBase> pistons_all = new List<IMyPistonBase>();
        List<IMyTextSurface> myTextSurfaces_d1 = new List<IMyTextSurface>();
        List<IMyTextSurface> myTextSurfaces_d2 = new List<IMyTextSurface>();
        List<IMyTextSurface> myTextSurfaces_d3 = new List<IMyTextSurface>();
        List<IMyTextSurface> myTextSurfaces_d4 = new List<IMyTextSurface>();
        string d1_tag = "";
        string d2_tag = "";
        string d3_tag = "";
        string d4_tag = "";
        string interface_display = "";
        string channel_tag_display = "";
        string secondary_tag_display = "";


        string commandArg1 = "shipname";
        string commandArg2 = "dronelength";
        string commandArg3 = "clearoffset";
        string commandArg4 = "rotatehome";
        string commandArg5 = "dronesperscreen";

        #endregion
        public void Save()
        {
            if (setupComplete)
            {
                sb = new StringBuilder();
                _ini.Clear();
                _ini.Set("configuration", "runargument", runargument);
                _ini.Set("configuration", "ship grid tag", secondary_tag);
                if (gridBoreFinished.Count > 0 && gridBoreOccupied.Count > 0)
                {
                    for (int i = 0; i < gridBoreFinished.Count; i++)
                    {

                        if (gridBoreFinished[i])
                        {
                            g1 = "1";
                        }
                        else
                        {
                            g1 = "0";
                        }
                        if (gridBoreOccupied[i])
                        {
                            g2 = "1";
                        }
                        else
                        {
                            g2 = "0";
                        }
                        px = gridBorePosition[i].X.ToString();
                        py = gridBorePosition[i].Y.ToString();
                        pz = gridBorePosition[i].Z.ToString();
                        sb.Append($"{g1}:{g2}:{px}:{py}:{pz}:;");
                    }
                    _ini.Set("jobdata", "gridstatus", sb.ToString());
                    Storage = _ini.ToString();
                    safebuilder.Clear();
                    //save jobdata into customdata
                    if (safebuilder.TryParse(Me.CustomData))
                    {
                        safebuilder.Set("configuration", "runargument", runargument);
                        safebuilder.Set("configuration", "ship grid tag", secondary_tag);
                        safebuilder.Set("jobdata", "gridstatus", sb.ToString());
                        Me.CustomData = safebuilder.ToString();
                    }
                    safebuilder.Clear();
                    sb.Clear();
                    _ini.Clear();
                }

            }


        }


        public void AntennaSaveData(IMyRadioAntenna block)
        {
            _antennaStore.Clear();
            if (_antennaStore.TryParse(block.CustomData.ToString()))
            {
                _antennaStore.Set("Configuration", "drone group tag", drone_tag);
                _antennaStore.Set("Configuration", "ship grid tag", secondary_tag);
            }
            else
            {
                _antennaStore.Set("Configuration", "drone group tag", drone_tag);
                _antennaStore.Set("Configuration", "ship grid tag", secondary_tag);
            }
            block.CustomData = _antennaStore.ToString();
            _antennaStore.Clear();
        }
        public void manageFirstLoad(string input, string datacommandinput)
        {
            if (!string.IsNullOrWhiteSpace(Storage) && !string.IsNullOrEmpty(Storage))
            {
                GetStoredData(Storage);
                Echo("Running first parse");
                ParseAndApplyArguments(runargument);
                Echo("Configuration loaded from Storage.");
            }
            else
            {
                GetStoredData(Storage);
                ParseAndApplyArguments(runargument);
                Echo("No Storage data found, configuration loaded from arguments or defaults.");
            }

        }
        public void Main(string argument, UpdateType updateSource)
        {
            //sbtexttemp.AppendLine("Running Modular Main - v1");
            int startInstructions = Runtime.CurrentInstructionCount;
            _frameCounter++;
            if (!string.IsNullOrEmpty(argument) && !string.IsNullOrWhiteSpace(argument) && !firstload)
            {
                // --- Argument takes precedence for setup and override ---
                runargument = argument;
                ParseAndApplyArguments(argument);
                Save();
                i_init = true;
                // Force a full setup if arguments changed
                setupComplete = false;
            }



            UpdateRuntimeMetrics(updateSource);
            InitializeSystem();
            firstload = false;
            if (!setupComplete)
            {
                Echo("Setup incomplete - exiting");
                ClearAllNonEmptyLists();
                return;
            }
            // Echo("M1");
            ProcessInputs(argument);
            //   Echo("M2");
            ManageCommunications();
            //   Echo("M3");
            UpdateMiningGrid();
            //    Echo("M4");
            HandleDroneOperations();
            //   Echo("M5");
            RenderDisplays();
            //   Echo("M6");
            UpdateStatus();
            //   Echo("M7");
            Echo(sbtexttemp.ToString());
            sbtexttemp.Clear();
            sbtexttemp.AppendLine($"Main Total: {Runtime.CurrentInstructionCount - startInstructions}");

        }

        private void UpdateRuntimeMetrics(UpdateType updateSource)
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            double _Runtime = Runtime.LastRunTimeMs;
            totalRuntimeMs += _Runtime;
            runCount++;
            if (runCount == 10)
            {
                // averageRuntimeMs = totalRuntimeMs / runCount;
                runCount = 0;
                totalRuntimeMs = 0;
            }
            //sbtexttemp.AppendLine($"UpdateRuntimeMetrics: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void InitializeSystem()
        {

            int startInstructions = Runtime.CurrentInstructionCount;
            if (!setupComplete)
            {

                SetupSystem();
                setupComplete = true;
                Echo("Setup complete!");
            }
            CheckSystemSetupStatus();
            if (!setupComplete)
            {
                Echo("Setup incomplete - exiting");
                ClearAllNonEmptyLists();
                return;
            }
            AntennaSaveData(antennaActual);
            sbtexttemp.AppendLine($"GMDC {ver} Running {icon} ");
            sbtexttemp.AppendLine($"Channel: {channel_tag_display} ");
            sbtexttemp.AppendLine($"Ship Name: {secondary_tag_display} ");
            sbtexttemp.AppendLine($"D1 Tag ({myTextSurfaces_d1.Count}): {d1_tag} ");
            sbtexttemp.AppendLine($"D2 Tag ({myTextSurfaces_d2.Count}: {d2_tag} ");
            sbtexttemp.AppendLine($"D2 DrnPS: (#{drones_per_screen}) ");
            sbtexttemp.AppendLine($"D3 Tag ({myTextSurfaces_d3.Count}): {d3_tag} ");
            sbtexttemp.AppendLine($"D4 Tag ({myTextSurfaces_d4.Count}): {d4_tag} ");
            sbtexttemp.AppendLine($"D4 Vis rotation: {rotateHome} ");
            sbtexttemp.AppendLine($"Clear offset: {drone_clear_offset}m ");
            sbtexttemp.AppendLine($"Drone length: {drone_length}m ");


            //sbtexttemp.AppendLine($"InitializeSystem: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ProcessInputs(string argument)
        {
            string blank = "";
            int startInstructions = Runtime.CurrentInstructionCount;
            ProcessInterface();
            //   Echo("M1a");
            HandleCommands(blank);
            //   Echo("M1b");
            //sbtexttemp.AppendLine($"ProcessInputs: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ManageCommunications()
        {
            int startInstructions = Runtime.CurrentInstructionCount;

            ProcessMessages();
            //sbtexttemp.AppendLine($"ManageCommunications: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateMiningGrid()
        {
            int startInstructions = Runtime.CurrentInstructionCount;

            InitializeMiningGrid();
            ValidateCustomData();
            ManageReload();
            PingDrones();
            if (rcdataOld != remoteControlActual.CustomData)
            {
                GetRemoteControlData(remoteControlActual.CustomData, remoteControlActual);
                rcdataOld = remoteControlActual.CustomData;
            }
            //sbtexttemp.AppendLine($"Pre-Prospect: valid= RC: {prospectTargetValid}  , ALN: {prospectAlignTargetValid}, coords= PB: {miningGPSCoordinates} RC: {targetGPSCoordinates}");
            if (prospectAlignTargetValid)
            {

            }
            if (prospectorMessageReceived)
            {


                Storage = null;
                prospectAlignTargetValid = false;
                customDataAlignTargetValid = false;
                GetRemoteControlData(remoteControlActual.CustomData, remoteControlActual);
                //sbtexttemp.AppendLine($"Post-Prospect: valid={prospectTargetValid}, {prospectAlignTargetValid}, coords={targetGPSCoordinates}");
                if (prospectAlignTargetValid)
                {
                    sbtexttemp.AppendLine($"Post-Prospect: Main PB: {miningCoordsValid} Align coords=:{alignGPSCoordinates}");
                }
                if (prospectTargetValid)
                {
                    sbtexttemp.AppendLine($"Formatting CustomData with: {targetGPSCoordinates.X}, {targetGPSCoordinates.Y}, {targetGPSCoordinates.Z}");
                    miningCoordinatesNew.Clear().AppendFormat("GPS:PDT:{0:0.##}:{1:0.##}:{2:0.##}:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:False:",
                        targetGPSCoordinates.X, targetGPSCoordinates.Y, targetGPSCoordinates.Z);
                    if (prospectAlignTargetValid)
                    {
                        miningCoordinatesNew.Append($"GPS:TGT:{alignGPSCoordinates.X}:{alignGPSCoordinates.Y}:{alignGPSCoordinates.Z}:#F77668:{safe_dstvl}:");
                    }
                    StoreRawInput(miningCoordinatesNew.ToString(), Me, gmdccategory, jobinfo);
                    //Me.CustomData = miningCoordinatesNew.ToString();
                }
                prospectorMessageReceived = false;
                gridCreated = false;
            }
            if (_oldCustomData != Me.CustomData)
            {
                GetCustomDataJobCommand(Me.CustomData, Me);
                _oldCustomData = Me.CustomData;
            }
            ProcessJobGrid();
            UpdateActiveDroneLimits();
            //sbtexttemp.AppendLine($"UpdateMiningGrid: {Runtime.CurrentInstructionCount - startInstructions}");
        }
        private void ManageReload()
        {
            if (loadsave)
            {
                gridCreated = false;
                i_init = true;
            }

        }
        private void HandleDroneOperations()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (Swarm.Count > 0 && gridCreated && timeDelayed)
            {
                UpdateDroneCounts();
                DroneUndockCheck();
                timeCountReset();
                ProcessRecallCommand();
                ProcessDroneState();
                update_display();
            }
            DroneResetStatusCounter();
            LightStatusManagement();
            //sbtexttemp.AppendLine($"HandleDroneOperations: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void RenderDisplays()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            DroneRenderCall(true);
            ListRenderCall();
            SpriteRenderCall();
            //sbtexttemp.AppendLine($"RenderDisplays: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateStatus()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            TimeCounterReset();
            if (_frameCounter % 10 == 0)
            {
                LocalStatusUpdate(Runtime.LastRunTimeMs);
            }
            if (_frameCounter > 10)
            {
                _frameCounter = 0;
            }
            //sbtexttemp.AppendLine($"UpdateStatus: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void LocalStatusUpdate(double _Runtime)
        {
            // Pre-calculate common factors if possible to simplify the append lines
            double tickRatio = (_Runtime / game_tick_length) * 100.0;

            sbtexttemp.Append("Load: ").Append(Math.Round(tickRatio, 3)).Append("% (")
                      .Append(Math.Round(_Runtime, 3)).Append("ms) S#:")
                      .Append(spriteCounter).Append(" ").Append(spriteInsert).Append('\n');

            sbtexttemp.Append("Drones #: ").Append(Swarm.Count).Append('\n');

            sbtexttemp.Append("Drone comms buffer: ").Append(droneMessagesBuffer.Count)
                      .Append(" OK: ").Append(droneMessageReceived).Append('\n');

            // Calculation for seconds: (ticks * tick_length) / 1000
            double broadcastSeconds = (((double)droneCommunicationsProcessingDelay * game_tick_length) / 1000.0) * (double)game_factor;
            sbtexttemp.Append("Cycles since last broadcast: ").Append(timeCounter)
                      .Append(" (").Append(Math.Round(broadcastSeconds, 1)).Append("s) ").Append(timeDelayed).Append('\n');

            double pingSeconds = (((double)droneCommunicationsPingDelay * game_tick_length) / 1000.0) * (double)game_factor;
            sbtexttemp.Append("Cycles since last ping: ").Append(dronePingTimerCount)
                      .Append(" (").Append(Math.Round(pingSeconds, 1)).Append("s)").Append('\n');

            double undockSecs = (((double)undockTimer * game_tick_length) / 1000.0) * (double)game_factor;
            double undockLimitSecs = (((double)undock_delay_limit * game_tick_length) / 1000.0) * (double)game_factor;
            sbtexttemp.Append("Undock cycle timer: ").Append(undockTimer)
                      .Append(" (").Append(Math.Round(undockSecs, 1)).Append("s) (")
                      .Append(Math.Round(undockLimitSecs, 1)).Append("s)").Append('\n');

            sbtexttemp.Append("Drones Undocking: ").Append(dronesUndocking).Append(" ").Append(total_drones_undocking).Append('\n');
            sbtexttemp.Append("Prospect comms buffer: ").Append(prospectorMessagesBuffer.Count).Append('\n');

            StateShifter();
        }

        private void TimeCounterReset()
        {
            timeCounter++;
            if (timeCounter >= droneCommunicationsProcessingDelay)
            {
                timeDelayed = true;
            }
            dronePingTimerCount++;
            if (dronePingTimerCount >= droneCommunicationsPingDelay && droneMessagesBuffer.Count <= 0)
            {
                dronesPinged = false;
            }
            if (dronesUndocking)
            {
                undockTimer++;
            }
            if (undockTimer > droneUndockDelayTime)
            {
                dronesUndocking = false;
            }
        }

        private void SpriteRenderCall()
        {
            //coroutine visual
            if (Visport_OK)
            {


                if (visCoroutine == null && !frame_generator_finished)
                {
                    visCoroutine = BuildSprites(miningGPSCoordinates, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid, rotateHome);
                }
                if (visCoroutine != null && !frame_generator_finished)
                {
                    // Check the current yield value
                    bool currentYield = visCoroutine.Current;

                    // If the coroutine is finished, you can perform completion logic
                    if (!visCoroutine.MoveNext())
                    {
                        // The coroutine has finished executing
                        sbtexttemp.AppendLine("Job rendering complete.");
                        visCoroutine?.Dispose();
                        visCoroutine = null;
                        BuildSprites(miningGPSCoordinates, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid, rotateHome).Dispose();
                    }
                    else
                    {
                        // Handle intermediate status if needed
                        if (!currentYield)
                        {
                            sbtexttemp.AppendLine($"Rendering mining job ... {Math.Round(percent_list_vis, 1)}%  {Math.Round(percent_list_drones, 1)}%");
                            visCoroutine.MoveNext();
                        }
                    }

                }
                if (display_tag_vis.Count > 0 && display_tag_vis[0] != null && gridBoreFinished.Count > 0 && frame_generator_finished)
                {
                    frame_generator_finished = false;
                    if (spriteCounter >= spriteCountLimit)
                    {

                        sV.DrawFrame();
                        var spritepinged = new MySprite();
                        sV.DrawFrame().Add(spritepinged);
                        if (sprites.Count == 0)
                        {
                            sV.DrawFrame().Dispose();
                        }
                    }
                    else
                    {
                        //sbtexttemp.AppendLine($"Frame reset - spritecount {spriteCounter}");
                        var frame = sV.DrawFrame();
                        DrawSprites(ref frame);
                        frame.Dispose();
                        sprites.Clear();

                    }
                    if (spriteCounter > spriteCountLimit + 1)
                    {
                        spriteCounter = 0;
                        spriteInsert = false;
                    }

                }
            }
        }

        private void ListRenderCall()
        {
            //coroutine list
            if (listCoroutine == null && !listGeneratorFinished)
            {
                listCoroutine = GenListDisplay();
            }
            if (listCoroutine != null && !listGeneratorFinished)
            {
                // Check the current yield value
                bool currentYield = listCoroutine.Current;

                // If the coroutine is finished, you can perform completion logic
                if (!listCoroutine.MoveNext())
                {
                    // The coroutine has finished executing
                    sbtexttemp.AppendLine("Mining list complete.");
                    listCoroutine?.Dispose();
                    listCoroutine = null;
                }
                else
                {
                    // Handle intermediate status if needed
                    if (!currentYield)
                    {
                        sbtexttemp.AppendLine($"Updating mining job list... {Math.Round(percent_list, 1)}%");
                        listCoroutine.MoveNext();
                    }
                }

            }
            if (listGeneratorFinished)
            {
                if (myTextSurfaces_d3.Count > 0)
                {
                    for (int i = 0; i < myTextSurfaces_d3.Count; i++)
                    {
                        if (myTextSurfaces_d3[i] != null)
                        {
                            myTextSurfaces_d3[i].WriteText(displayTextList.ToString());
                        }
                    }
                }
                listGeneratorFinished = false;
                displayTextList.Clear();
                listHeaderGenerated = false;
            }
        }

        private void LightStatusManagement()
        {
            if (lightIndicatorActual == null || lightsTag[0] == null)
            {
                sbtexttemp.AppendLine($"Indicator light missing {lightsTagName.Replace("[", "[[").Replace("]", "]]")} - early exit");
                return;
            }
            if (canTransmit && !readyFlag || commandAsk == "Init")
            {
                lightIndicatorActual.SetValue("Color", Cred);
                if (commandAsk == "Init")
                {
                    lightIndicatorActual.SetValue("Color", Cgreen);
                }
                lightIndicatorActual.Enabled = true;
                lightIndicatorActual.BlinkIntervalSeconds = 0.7f;
                lightIndicatorActual.BlinkLength = 20.0f;
                lightIndicatorActual.Enabled = true;
                screenStatus = "Not Ready";
            }
            if (canTransmit && readyFlag)
            {
                lightIndicatorActual.SetValue("Color", Cgreen);
                lightIndicatorActual.BlinkIntervalSeconds = 0;
                lightIndicatorActual.BlinkLength = 10.0f;
                lightIndicatorActual.Enabled = true;
                screenStatus = "Ready";
            }
            if (!canTransmit && commandAsk == "Stop" || commandAsk == "" || commandAsk == "Freeze" || commandAsk == "Eject" || commandAsk == "Recall")
            {
                lightIndicatorActual.SetValue("Color", Cred);
                if (commandAsk == "Eject")
                {
                    lightIndicatorActual.SetValue("Color", Cyellow);
                }

                if (commandAsk == "Recall")
                {
                    lightIndicatorActual.SetValue("Color", Cblue);
                }
                lightIndicatorActual.BlinkIntervalSeconds = 0.7f;
                lightIndicatorActual.BlinkLength = 20.0f;
                lightIndicatorActual.Enabled = true;
                screenStatus = "Not Ready";
            }

            if (totalDronesActive > 0 && gridBoresCompleted < totalMiningRuns && canTransmit && canRun || faultLightOutput)
            {
                lightIndicatorActual.BlinkIntervalSeconds = 0;
                if (totalDronesUnknown > 0)
                {
                    lightIndicatorActual.BlinkIntervalSeconds = 0.7f;
                }
                lightIndicatorActual.SetValue("Color", Cyellow);
                if (totalDronesDamaged > 0)
                {
                    lightIndicatorActual.BlinkIntervalSeconds = 0.7f;
                    lightIndicatorActual.SetValue("Color", Coren);
                }
                if (faultLightOutput)
                {
                    lightIndicatorActual.BlinkIntervalSeconds = 0;
                    lightIndicatorActual.SetValue("Color", Coren);
                }

                lightIndicatorActual.Enabled = true;
                screenStatus = "Working";
            }
            if (gridBoresCompleted >= totalMiningRuns)
            {
                lightIndicatorActual.SetValue("Color", Cred);
                lightIndicatorActual.Enabled = true;
                screenStatus = "Sequence Finished";
            }
        }

        private void DroneResetStatusCounter()
        {
            if (Swarm.Count > 0 && canReset)
            {
                droneResetStatusCount = CountIntegerValues("GpsListPosition", -1);
                droneDockedStatusCount = CountStatusValues("ControlStatus", "Docked");
            }
            if (Swarm.Count > 0)
            {
                if (droneResetStatusCount == Swarm.Count && droneDockedStatusCount == Swarm.Count && canReset)
                {
                    readyFlag = true;
                }
            }
        }

        private void ProcessDroneState()
        {
            DroneData drone = null;

            #region drone_state_machine_management
            if (!string.IsNullOrEmpty(incomingName) && droneMessageConfirmed)
            {
                cm = "0"; //Reset command on message entry
                if (Swarm.ContainsKey(incomingName))
                {
                    drone = Swarm[incomingName];
                }
                int i = receivedDroneNameIndex;

                if (canInit || canReset || drone.ResetFunction || canLoading)
                {
                    generalReset = true;
                }
                else generalReset = false;
                faultCounter = CountTrueValues("Dst");

                if (faultCounter < Swarm.Count)
                {
                    faultLightOutput = true;
                }
                else faultLightOutput = false;

                //recall sequence reset - global
                if (!drone.RecallList || canReset || canInit || canLoading)
                {
                    drone.RecallSequence = 0;
                }
                displayTextMain.Clear();

                if (drone.GpsListPosition > -1 && !drone.AssignedCoordinates)
                {
                    drone.GpsListPosition = -1;
                }
                if (Swarm.Count > 0)
                {
                    if (drone.GpsListPosition > -1 && drone.AssignedCoordinates && drone.ControlStatus.Contains("Docked") && drone.Docked == "True" && drone.IsMining && drone.IsReady)
                    {
                        if (gridBoreFinished[drone.GpsListPosition])
                        {
                            drone.GpsListPosition = -1;
                            drone.IsMining = false;
                            drone.AssignedCoordinates = false;
                        }

                    }
                }
                //if undocked request local recall sequence flag to ON
                if (drone.GpsListPosition == -1 && !drone.AssignedCoordinates && drone.Undocked == "True" && drone.Docked == "False" && !drone.RecallList && !mustUndockCommand || drone.GpsListPosition == -1 && !drone.AssignedCoordinates && drone.Undocked == "False" && drone.Docked == "False" && !drone.RecallList && !mustUndockCommand)
                {
                    drone.RecallList = true;
                }
                if (drone.RecallList)
                {
                    droneTXRecallChannel = drone.Name + " " + commandRecall;
                    IGC.SendBroadcastMessage(droneTXRecallChannel, commandRecall, TransmissionDistance.TransmissionDistanceMax);
                }
                if (!drone.RecallList)
                {
                    droneTXRecallChannel = drone.Name + " " + commandRecall;
                    IGC.SendBroadcastMessage(droneTXRecallChannel, commandOperate, TransmissionDistance.TransmissionDistanceMax);
                }


                if (drone.ControlStatus.Contains("Docked") && drone.GpsListPosition == -1 && drone.IsMining && (drone.ControlSequence == 0 || drone.ControlSequence == 8))
                {
                    drone.IsMining = false;
                }
                if ((totalDronesMining) >= boresRemaining && !drone.IsMining && gridBoresCompleted <= totalMiningRuns || boresRemaining == 0 && !drone.IsMining)
                {
                    if (!dronesLaunchedStatus || dronesUndocking)
                    {
                        drone.MustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive > maxActiveDronesCount || dronesUndocking)
                    {
                        drone.MustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive <= maxActiveDronesCount)
                    {
                        drone.MustWait = false;
                    }
                }
                else if ((totalDronesMining) < boresRemaining && gridBoresCompleted < totalMiningRuns || drone.IsMining && (totalDronesMining) <= boresRemaining)
                {
                    if (!dronesLaunchedStatus)
                    {
                        drone.MustWait = false;
                    }
                    if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount)
                    {
                        drone.MustWait = false;
                    }

                    if (dronesLaunchedStatus && totalDronesActive > maxActiveDronesCount || dronesUndocking)
                    {
                        drone.MustWait = true;
                    }
                }
                if (drone.GpsListPosition == -1 && (totalDronesMining) >= boresRemaining || dronesUndocking)
                {
                    if (!dronesLaunchedStatus)
                    {
                        drone.MustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive >= maxActiveDronesCount || dronesUndocking)
                    {
                        drone.MustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount || dronesUndocking)
                    {
                        drone.MustWait = true;
                    }
                }
                if (drone.GpsListPosition > -1 && drone.GpsListPosition < gridBorePosition.Count)
                {
                    if (gridBoreOccupied[drone.GpsListPosition] && !drone.IsMining)
                    {
                        if (!dronesLaunchedStatus || dronesUndocking)
                        {
                            drone.MustWait = true;

                        }
                        if (dronesLaunchedStatus && totalDronesActive >= maxActiveDronesCount || dronesUndocking)
                        {
                            drone.MustWait = true;
                        }
                        if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount || dronesUndocking)
                        {
                            drone.MustWait = true;
                        }
                    }
                    else if (gridBoresCompleted < totalMiningRuns && !gridBoreOccupied[drone.GpsListPosition] && !gridBoreFinished[drone.GpsListPosition] && !drone.IsMining)
                    {
                        if (!dronesLaunchedStatus)
                        {
                            drone.MustWait = false;
                        }
                        if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount)
                        {
                            drone.MustWait = false;
                        }
                        if (dronesLaunchedStatus && totalDronesActive >= maxActiveDronesCount || dronesUndocking)
                        {
                            drone.MustWait = true;
                        }
                    }
                    if (!gridBoreFinished[drone.GpsListPosition])
                    {
                        int queued_count = CountIntegerValues("GpsListPosition", drone.GpsListPosition);
                        if (gridBoreOccupied[drone.GpsListPosition] && queued_count == 0)
                        {
                            gridBoreOccupied[drone.GpsListPosition] = false;
                        }
                    }
                }

                updateDisplay(drone);

                gpsGridPositionValue = drone.GpsListPosition;
                if (drone.ControlStatus == "Docked Idle")
                {
                    drone.IsReady = true;
                }
                if (drone.ControlStatus.Contains("Recharging") || drone.ControlStatus.Contains("Unloading"))
                {
                    drone.IsReady = false;
                }
                if (drone.ControlStatus.Contains("Idle") && drone.Undocked == "True" && drone.Docked == "False" && boresRemaining == 0 && !drone.RecallList)
                {
                    drone.RecallList = true;
                }
                if (drone.GpsListPosition == -1 && drone.AssignedCoordinates && drone.Docked == "True" && drone.ControlStatus.Contains("Docked"))
                {
                    drone.AssignedCoordinates = false;
                }
                if (drone.GpsListPosition > -1) // attempting to reset droneGPSListPosition here if bore is finished - attempt here
                {
                    if (drone.ControlSequence == 0 && drone.IsReady && drone.TunnelFinished == "False" && drone.ControlStatus.Contains("Docked") && gridBoreFinished[drone.GpsListPosition] && drone.IsMining && drone.AssignedCoordinates && canRun)
                    {
                        drone.IsMining = false;
                        drone.AssignedCoordinates = false;
                        gridBoreOccupied[drone.GpsListPosition] = true; // cant guarantee this is occupied, might be occupied by another grid and not reported finished yet
                        drone.GpsListPosition = -1;
                        gpsGridPositionValue = -1;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();
                        if (canTransmit && drone.TransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.TransmissionStatus = false;
                        }
                    }
                    if (drone.ControlSequence > 0 && drone.Docked == "True" && drone.ControlStatus.Contains("Docked") && !drone.AssignedCoordinates && drone.GpsListPosition > -1 && canRun)
                    {
                        drone.IsMining = false;
                        drone.ControlSequence = 0; //reset sequence if docked and not assigned
                        drone.GpsListPosition = -1;
                        gpsGridPositionValue = -1;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();
                        if (canTransmit && drone.TransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.TransmissionStatus = false;
                        }
                    }

                }
                if (drone.IsReady && drone.TunnelFinished == "False" && drone.Docked == "True" && canRun && !drone.AssignedCoordinates && drone.ControlSequence == 0 && !drone.MustWait && !drone.IsMining && !disableRunArgument && miningGridValid)
                {
                    if (gridBoresCompleted < totalMiningRuns && miningGridValid && !drone.AssignedCoordinates && !drone.MustWait)
                    {

                        if (gridBoreFinished.Count > 0)
                        {
                            if (skipBoresNumber > gridBoreFinished.Count)
                            {
                                skipBoresNumber = 0;
                            }
                            if (skipBoresNumber > 0)
                            {
                                for (int j = 0; j < skipBoresNumber; j++)
                                {
                                    if (j > gridBoreFinished.Count - 1 || j > skipBoresNumber - 1)
                                    {
                                        break;
                                    }
                                    gridBoreFinished[j] = true;
                                }
                            }
                            for (int k = 0; k < gridBoreFinished.Count; k++)
                            {

                                if (k > gridBoreFinished.Count - 1)
                                {
                                    k = gridBoreFinished.Count - 1;
                                }
                                if (!gridBoreFinished[k])
                                {
                                    if (Swarm.Count > 0)
                                    {
                                        int queued_count = CountIntegerValues("GpsListPosition", k);
                                        if (!gridBoreOccupied[k] && queued_count > 0) //check if preassigned here
                                        {
                                            gridBoreOccupied[k] = true;
                                        }
                                    }
                                }
                                if (!gridBoreFinished[k] && !gridBoreOccupied[k])
                                {
                                    currentGPSIndex = k;
                                    break;
                                }
                            }
                        }
                        realGPSIndex = currentGPSIndex;
                        if (gpsGridPositionValue == -1)
                        {
                            gpsGridPositionValue = currentGPSIndex;
                            drone.GpsCoordinates = gridBorePosition[gpsGridPositionValue];
                            drone.GpsListPosition = gpsGridPositionValue;
                        }
                        else
                        {
                            gpsGridPositionValue = drone.GpsListPosition;
                            if (gpsGridPositionValue > -1 && gpsGridPositionValue < gridBorePosition.Count)
                            {
                                drone.GpsCoordinates = gridBorePosition[gpsGridPositionValue];
                            }

                        }
                        if (!miningGridValid)
                        {
                            totalMiningRuns = 1;
                            if (Swarm.Count > 0)
                            {
                                drone.GpsCoordinates = miningGPSCoordinates;
                            }
                            gpsGridPositionValue = 0;
                            currentGPSIndex = 0;
                        }
                        //suspect code here
                        sbtexttemp.AppendLine($"Drone coords: {i}");
                        if (i < Swarm.Count)
                        {
                            drone.AssignedCoordinates = true;
                            sbtexttemp.AppendLine($"Drone coords assigned: {i} {drone.AssignedCoordinates}");
                        }
                    }
                    else if (!miningGridValid)
                    {
                        totalMiningRuns = 1;
                        drone.GpsCoordinates = miningGPSCoordinates;
                        drone.AssignedCoordinates = true;
                        gpsGridPositionValue = 0;
                        currentGPSIndex = 0;
                        sbtexttemp.AppendLine("invalid grid - defaulting");
                    }
                    sbtexttemp.AppendLine("data staging");
                    if (drone.GpsListPosition > -1)
                    {
                        if (gridBoreOccupied[drone.GpsListPosition] && !drone.IsMining)
                        {
                            drone.MustWait = true;
                        }
                        else if ((totalDronesMining) < boresRemaining && gridBoresCompleted < totalMiningRuns || !gridBoreOccupied[drone.GpsListPosition] && !gridBoreFinished[drone.GpsListPosition] && !drone.IsMining)
                        {
                            drone.MustWait = false;
                        }
                        if (gridBoresCompleted != totalMiningRuns && !drone.MustWait)
                        {
                            drone.ControlSequence = 1;
                            drone.IsMining = true;
                            gridBoreOccupied[drone.GpsListPosition] = true;
                        }
                        else
                        {
                            drone.ControlSequence = 0;
                            drone.IsMining = false;
                        }

                        if (gridBoreFinished[drone.GpsListPosition])
                        {
                            //suspect coordinates here 2
                            sbtexttemp.AppendLine($"Drone position finished {i}");
                            drone.ControlSequence = 0;
                            drone.IsMining = false;
                            drone.AssignedCoordinates = false;
                            drone.GpsListPosition = -1;
                            gpsGridPositionValue = -1;
                        }
                    }
                }


                tx_chan = drone.Name;
                cd1 = gpsGridPositionValue.ToString();
                cm = "0";
                xp = Math.Round(drone.GpsCoordinates.X, 2).ToString();
                yp = Math.Round(drone.GpsCoordinates.Y, 2).ToString();
                zp = Math.Round(drone.GpsCoordinates.Z, 2).ToString();
                cd5 = customData5;
                cd6 = (drillLength + safe_dstvl).ToString();
                igd = (ignoreDepth + safe_dstvl + drone_length - drone_clear_offset).ToString();
                if (prospectAlignTargetValid || customDataAlignTargetValid)
                {
                    xp2 = Math.Round(((drone.GpsCoordinates.X - miningGPSCoordinates.X) + alignGPSCoordinates.X), 2).ToString();
                    yp2 = Math.Round(((drone.GpsCoordinates.Y - miningGPSCoordinates.Y) + alignGPSCoordinates.Y), 2).ToString();
                    zp2 = Math.Round(((drone.GpsCoordinates.Z - miningGPSCoordinates.Z) + alignGPSCoordinates.Z), 2).ToString();
                }
                else
                {
                    xp2 = "";
                    yp2 = "";
                    zp2 = "";
                }
                if (drone.ControlSequence == 1 && drone.AssignedCoordinates && !drone.MustWait && !disableRunArgument || drone.ControlSequence == 2 && drone.ControlStatus == "Docked Idle" && drone.Docked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.canlaunch = true;

                    drone.IsMining = true;
                    if (drone.GpsListPosition > -1)
                    {
                        gridBoreOccupied[drone.GpsListPosition] = true;
                    }
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "7";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (drone.AssignedGates.Count > 0)
                    {
                        for (int g = 0; g < drone.AssignedGates.Count; g++)
                        {
                            if (drone.AssignedGates[g] != null)
                            {
                                if (!drone.ControlStatus.Contains("Docked"))
                                {
                                    if (!drone.AssignedGates[g].Enabled)
                                    {
                                        drone.AssignedGates[g].Enabled = true;
                                    }
                                    if (drone.AssignedGates[g].Status == DoorStatus.Closed || drone.AssignedGates[g].Status == DoorStatus.Closing)
                                    {
                                        drone.AssignedGates[g].OpenDoor();
                                    }
                                    if (drone.AssignedGates[g].Status != DoorStatus.Open)
                                    {
                                        drone.canlaunch = false;
                                    }
                                }
                                else if (drone.Docked == "True" && drone.ControlStatus.Contains("Docked"))
                                {
                                    if (drone.AssignedGates[g].Status == DoorStatus.Open || drone.AssignedGates[g].Status == DoorStatus.Opening)
                                    {
                                        drone.AssignedGates[g].CloseDoor();
                                    }
                                    if (drone.AssignedGates[g].Status != DoorStatus.Closed)
                                    {
                                        drone.canlaunch = false;
                                    }
                                }
                            }
                        }
                    }
                    if (drone.canlaunch)
                    {
                        drone.ControlSequence = 2;
                        if (canTransmit && drone.TransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.TransmissionStatus = false;
                        }
                    }

                }

                if (drone.ControlSequence == 2 && drone.ControlStatus == "Undocked" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument || drone.ControlSequence == 2 && drone.ControlStatus == "Docking" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 3;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 2 && drone.ControlStatus == "Undocking" && drone.Docked == "False" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument && drone.Dcs <= bclu)
                {
                    drone.ControlSequence = 13;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 8 && drone.ControlStatus.Contains("RTB Ready") && drone.Docked == "False" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    if (drone.GpsListPosition > -1)
                    {
                        gridBoreOccupied[drone.GpsListPosition] = false; //trying to do this for mining efficiency - clear occupation when safely exited

                    }
                    gpsGridPositionValue = -1; //unassign drone.GpsListPosition from drone here if finished mining and in safe position

                    drone.ControlSequence = 13;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 13 && drone.ControlStatus == "Idle" && drone.Docked == "False" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument || drone.ControlSequence == 5 && drone.ControlStatus == "Docking" && drone.Docked == "False" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument && drone.Dcs <= bclu)
                {
                    drone.ControlSequence = 8;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "6";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 3 && drone.ControlStatus == "Idle" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 4;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "4";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 4 && drone.ControlStatus == "Nav End" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 5;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 4 && drone.ControlStatus == "Docked Idle" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 5 && drone.ControlStatus == "Idle" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 6;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "2";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 6 && drone.ControlStatus == "Nav End" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 7;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 7 && drone.ControlStatus == "Idle" && drone.Undocked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 8;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "5";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }

                }

                if (drone.ControlSequence >= 8 && drone.ControlStatus.Contains("Dock") && drone.IsMining || drone.ControlSequence == 4 && drone.ControlStatus.Contains("Docked") && drone.IsMining)
                {
                    if (drone.GpsListPosition > -1)
                    {
                        gridBoreOccupied[drone.GpsListPosition] = false;
                    }
                }

                if (drone.ControlSequence >= 8 && (drone.ControlStatus.Contains("Dock") || drone.ControlStatus.Contains("Exit") || drone.ControlStatus.Contains("RTB")) && drone.IsMining && drone.AssignedCoordinates && drone.IsMining && drone.TunnelFinished == "True")
                {
                    if (drone.GpsListPosition > -1)
                    {
                        if (!gridBoreFinished[drone.GpsListPosition])
                        {
                            gridBoreFinished[drone.GpsListPosition] = true; //Finish bore here
                            sbtexttemp.AppendLine($"Grid bore finished: {drone.GpsListPosition}");
                        }
                    }
                }


                if (drone.ControlSequence == 8 && drone.IsReady && drone.Docked == "True" && (drone.TunnelFinished == "False" && drone.GpsListPosition > -1 && drone.GpsListPosition < gridBoreFinished.Count) && drone.AssignedCoordinates && !disableRunArgument)
                {
                    drone.ControlSequence = 1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 8 && !drone.IsReady && drone.Docked == "True" && drone.TunnelFinished == "False" && drone.AssignedCoordinates && !disableRunArgument || drone.ControlSequence == 8 && !drone.IsReady && drone.Docked == "True" && drone.TunnelFinished == "True" && drone.AssignedCoordinates && !disableRunArgument || drone.ControlSequence >= 1 && drone.ControlSequence <= 4 && !drone.IsReady && drone.Docked == "True" && drone.TunnelFinished == "False" && drone.AssignedCoordinates && !disableRunArgument)
                {
                    drone.ControlSequence = 0;
                    drone.AssignedCoordinates = false;
                    drone.IsMining = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 8 && drone.IsReady && drone.IsMining && drone.Docked == "True" && (drone.TunnelFinished == "True") && drone.AssignedCoordinates && !disableRunArgument)
                {
                    drone.ControlSequence = 9;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }



                if (drone.ControlSequence == 8 && drone.IsReady && drone.Docked == "True" && drone.GpsListPosition == -1 && !drone.AssignedCoordinates && !disableRunArgument)
                {
                    drone.ControlSequence = 0;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlSequence == 9 && drone.IsReady && drone.Docked == "True" && (drone.TunnelFinished == "True") && canRun && drone.AssignedCoordinates && !disableRunArgument || drone.ControlSequence == 9 && drone.IsReady && drone.Docked == "True" && (drone.TunnelFinished == "True") && (!drone.AssignedCoordinates) && !disableRunArgument)
                {
                    drone.ControlSequence = 10;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }


                if (drone.ControlSequence == 10 && drone.IsReady && drone.Docked == "True" && (drone.TunnelFinished == "True") && generalReset && drone.AssignedCoordinates && !disableRunArgument || drone.ControlSequence == 10 && drone.IsReady && drone.Docked == "True" && (drone.TunnelFinished == "True") && drone.AssignedCoordinates && !disableRunArgument || drone.ControlSequence == 0 && drone.IsReady && drone.Docked == "True" && (drone.TunnelFinished == "True") && drone.AssignedCoordinates && !disableRunArgument)
                {
                    drone.ControlSequence = 11;
                    totalMiningSequencesComplete++;
                    gpsGridPositionValue = -1;
                    drone.ResetFunction = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "8";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }

                }


                if (drone.ControlSequence == 11 && drone.IsReady && drone.Docked == "True" && drone.TunnelFinished == "False" && drone.AssignedCoordinates && totalMiningSequencesComplete <= totalMiningRuns && miningGridValid && !disableRunArgument)
                {
                    drone.ControlSequence = 0;
                    drone.AssignedCoordinates = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }

                }

                if (drone.ControlSequence == 11 && drone.ControlStatus.Contains("Docked") && drone.Docked == "True" && drone.TunnelFinished == "False" && currentGPSIndex < totalMiningRuns && drone.AssignedCoordinates && totalMiningSequencesComplete > totalMiningRuns && !disableRunArgument || drone.ControlSequence == 11 && drone.IsReady && drone.Docked == "True" && drone.TunnelFinished == "False" && drone.AssignedCoordinates && miningGridValid == false && totalMiningSequencesComplete >= totalMiningRuns && !disableRunArgument)
                {
                    drone.ControlSequence = 12;
                    drone.AssignedCoordinates = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }

                    displayTextMain.Append('\n');
                    displayTextMain.Append("Mining seq. complete");
                }
                if (drone.ControlSequence == 12 && drone.ControlStatus.Contains("RTB") && drone.Docked == "False" && drone.TunnelFinished == "True" && !disableRunArgument)
                {
                    drone.AssignedCoordinates = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }
                if (drone.ControlSequence == 12 && drone.ControlStatus.Contains("Idle") && drone.Docked == "False" && !drone.AssignedCoordinates && !disableRunArgument)
                {
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "6";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlStatus.Contains("Docked") && drone.Docked == "True" && drone.TunnelFinished == "True" && generalReset || drone.ControlStatus.Contains("Docked") && drone.Docked == "True" && drone.TunnelFinished == "True" && generalReset && !disableRunArgument)
                {
                    drone.ControlSequence = 0;
                    totalMiningSequencesComplete = 0;
                    drone.AssignedCoordinates = false;
                    drone.IsMining = false;
                    currentGPSIndex = 0;
                    gpsGridPositionValue = -1;
                    drone.ResetFunction = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "8";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (drone.ControlStatus.Contains("Docked") && drone.Docked == "True" && drone.TunnelFinished == "False" && generalReset && drone.ControlSequence == 0 && !disableRunArgument || drone.ControlStatus.Contains("Docked") && drone.Docked == "True" && drone.TunnelFinished == "False" && generalReset && !disableRunArgument || drone.ControlSequence == 6 && drone.ControlStatus == "Docked Idle" && drone.Docked == "True" && drone.AssignedCoordinates && drone.IsMining && !disableRunArgument)
                {
                    drone.ControlSequence = 0;
                    totalMiningSequencesComplete = 0;
                    drone.AssignedCoordinates = false;
                    drone.IsMining = false;
                    currentGPSIndex = 0;
                    gpsGridPositionValue = -1;
                    drone.ResetFunction = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.TransmissionOutput = c.ToString();
                    if (canTransmit && drone.TransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.TransmissionStatus = false;
                    }
                }

                if (mustRecall_Command && !drone.RecallList && !mustUndockCommand && !canRun)
                {
                    drone.RecallList = true;
                }
                if (drone.RecallList)
                {
                    drone.canlaunch = true;
                    if (drone.RecallSequence == 0 && drone.ControlStatus == "Idle" || drone.RecallSequence == 0 && drone.ControlStatus == "Undocked" || drone.RecallSequence == 0 && drone.ControlStatus == "Nav" || drone.RecallSequence == 0 && drone.ControlStatus == "Undocking" || drone.RecallSequence == 0 && drone.ControlStatus == "Docking" || drone.RecallSequence == 0 && drone.ControlStatus == "Initiating mining" || drone.RecallSequence == 0 && drone.ControlStatus.Contains("RTB"))
                    {
                        drone.RecallSequence = 1;
                        if (drone.ControlSequence > 0)
                        {
                            drone.ControlSequence = 0;
                        }

                    }
                    if (drone.AssignedGates.Count > 0)
                    {
                        for (int g = 0; g < drone.AssignedGates.Count; g++)
                        {
                            if (drone.AssignedGates[g] != null)
                            {
                                if (!drone.ControlStatus.Contains("Docked"))
                                {
                                    if (!drone.AssignedGates[g].Enabled)
                                    {
                                        drone.AssignedGates[g].Enabled = true;
                                    }
                                    if (drone.AssignedGates[g].Status == DoorStatus.Closed || drone.AssignedGates[g].Status == DoorStatus.Closing)
                                    {
                                        drone.AssignedGates[g].OpenDoor();
                                    }
                                    if (drone.AssignedGates[g].Status != DoorStatus.Open)
                                    {
                                        drone.canlaunch = false;
                                    }
                                }
                                else if (drone.Docked == "True" && drone.ControlStatus.Contains("Docked"))
                                {
                                    if (drone.AssignedGates[g].Status == DoorStatus.Open || drone.AssignedGates[g].Status == DoorStatus.Opening)
                                    {
                                        drone.AssignedGates[g].CloseDoor();
                                    }
                                    if (drone.AssignedGates[g].Status != DoorStatus.Closed)
                                    {
                                        drone.canlaunch = false;
                                    }
                                }
                            }
                        }
                    }
                    if (drone.RecallSequence == 0 && drone.ControlStatus == "Nav End")
                    {
                        drone.RecallSequence = 3;
                    }
                    if (drone.RecallSequence == 1)
                    {
                        drone.RecallSequence = 2;
                        drone.ControlSequence = 0;
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();
                    }
                    if (drone.RecallSequence == 2 && drone.ControlStatus == "Idle")
                    {
                        drone.RecallSequence = 3;
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "1";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();

                    }
                    if (drone.RecallSequence == 3 && drone.ControlStatus == "Nav End")
                    {
                        drone.RecallSequence = 4;
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();

                    }
                    if (drone.RecallSequence == 3 && drone.ControlStatus == "Nav" && drone.GpsListPosition == -1 || drone.RecallSequence == 3 && drone.ControlStatus == "Idle" && drone.GpsListPosition >= -1)
                    {
                        drone.RecallSequence = 4;
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();

                    }

                    if (drone.RecallSequence == 4 && drone.ControlStatus == "Idle")
                    {

                        drone.RecallSequence = 5;
                        drone.ControlSequence = 0;
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "6";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();

                    }
                    if (drone.RecallSequence == 4 && drone.ControlStatus == "Idle")
                    {
                        drone.RecallSequence = 5;
                        drone.ControlSequence = 0;
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "6";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();

                    }
                    if (drone.RecallSequence == 5 && drone.ControlStatus.Contains("Docked") || drone.RecallSequence == 0 && drone.ControlStatus.Contains("Docked"))
                    {
                        drone.RecallSequence = 0;
                        drone.AssignedCoordinates = false;
                        drone.ControlSequence = 0;
                        drone.RecallList = false;
                        drone.IsMining = false;
                        gpsGridPositionValue = -1;
                        drone.ResetFunction = true;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();

                    }
                    if (drone.canlaunch)
                    {
                        if (canTransmit && drone.TransmissionStatus)
                        {

                            transmitToDrone(drone);
                            drone.TransmissionStatus = false;
                        }
                    }
                }

                if (mustUndockCommand && !canRun)
                {
                    drone.canlaunch = true;
                    if (drone.ControlStatus == "Docked Idle")
                    {

                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "7";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();
                        if (drone.AssignedGates.Count > 0)
                        {
                            for (int g = 0; g < drone.AssignedGates.Count; g++)
                            {
                                if (drone.AssignedGates[g] != null)
                                {
                                    if (drone.ControlStatus.Contains("Docked"))
                                    {
                                        if (!drone.AssignedGates[g].Enabled)
                                        {
                                            drone.AssignedGates[g].Enabled = true;
                                        }
                                        if (drone.AssignedGates[g].Status == DoorStatus.Closed || drone.AssignedGates[g].Status == DoorStatus.Closing)
                                        {
                                            drone.AssignedGates[g].OpenDoor();
                                        }
                                        if (drone.AssignedGates[g].Status != DoorStatus.Open)
                                        {
                                            drone.canlaunch = false;
                                        }
                                    }
                                    else if (drone.Undocked == "True" && drone.ControlStatus.Contains("Undocked"))
                                    {
                                        if (drone.AssignedGates[g].Status == DoorStatus.Open || drone.AssignedGates[g].Status == DoorStatus.Opening)
                                        {
                                            drone.AssignedGates[g].CloseDoor();
                                        }
                                        if (drone.AssignedGates[g].Status != DoorStatus.Closed)
                                        {
                                            drone.canlaunch = false;
                                        }
                                    }
                                }
                            }
                        }
                        if (drone.canlaunch)
                        {
                            if (canTransmit && drone.TransmissionStatus)
                            {
                                transmitToDrone(drone);
                                drone.TransmissionStatus = false;
                            }
                        }

                    }
                    else
                    {
                        if (drone.AssignedGates.Count > 0)
                        {
                            for (int g = 0; g < drone.AssignedGates.Count; g++)
                            {
                                if (drone.AssignedGates[g] != null)
                                {
                                    if (drone.Undocked == "True" && drone.ControlStatus.Contains("Undocked"))
                                    {
                                        if (drone.AssignedGates[g].Status == DoorStatus.Open || drone.AssignedGates[g].Status == DoorStatus.Opening)
                                        {
                                            drone.AssignedGates[g].CloseDoor();
                                        }
                                        if (drone.AssignedGates[g].Status != DoorStatus.Closed)
                                        {
                                            drone.canlaunch = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (mustFreeze_Command)
                {
                    if (drone.ControlStatus == "Undocked" || drone.ControlStatus == "Idle")
                    {
                        gpsGridPositionValue = drone.GpsListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.TransmissionOutput = c.ToString();
                        if (canTransmit && drone.TransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.TransmissionStatus = false;
                        }
                    }
                }
                droneMessageConfirmed = false;
                receivedDroneNameIndex = -1;


                if (droneMessagesBuffer.Count > 0)
                {
                    droneMessagesBuffer.RemoveAt(0);
                }


            }
            #endregion
        }

        private void ProcessRecallCommand()
        {
            if (mustRecall_Command)
            {
                IGC.SendBroadcastMessage(tx_recall_channel, commandRecall, TransmissionDistance.TransmissionDistanceMax);
            }
            else
            {
                IGC.SendBroadcastMessage(tx_recall_channel, commandOperate, TransmissionDistance.TransmissionDistanceMax);
            }
            if (mustRecall_Command && currentGPSIndex > 0)
            {
                currentGPSIndex = 0;
            }
        }

        private void timeCountReset()
        {
            timeDelayed = false;
            timeCounter = 0;
        }

        private void UpdateActiveDroneLimits()
        {
            #region active_drones_processing
            //drone limit processing
            if (Swarm.Count > 0)
            {
                maxActiveDronesCount = Swarm.Count - dronesInFlightFactor;
                if (maxActiveDronesCount <= 1)
                {
                    maxActiveDronesCount = 1;
                }
                if (maxActiveDronesCount > dronesActiveHardLimit)
                {
                    maxActiveDronesCount = dronesActiveHardLimit;
                }
            }
            #endregion
        }

        public void writeInterfaceCommand(IMyTerminalBlock block, string command)
        {
            _interfaceCommand.Clear();
            if (_interfaceCommand.TryParse(block.CustomData.ToString()))
            {
                _interfaceCommand.Set("GMDIJobData", "interfacecommand", command);
            }
            else
            {
                _interfaceCommand.Set("GMDIJobData", "interfacecommand", command);
            }
            block.CustomData = _interfaceCommand.ToString();
            _interfaceCommand.Clear();
        }
        public void readInterfaceCommand(IMyTerminalBlock block)
        {
            var str = "";
            _interfaceCommand.Clear();
            if (_interfaceCommand.TryParse(block.CustomData.ToString()))
            {
                str = _interfaceCommand.Get("GMDIJobData", "interfacecommand").ToString();
                interfaceArgument = str;
            }
            else
            {
                interfaceArgument = "";
                writeInterfaceCommand(block, "");
            }
            _interfaceCommand.Clear();
        }
        private void ProcessJobGrid()
        {
            if (pbInterfaceActual == null || interfacePBTag[0] == null)
            {
                sbtexttemp.AppendLine($"Interface PB not found {interface_display}");
            }
            #region job_grid_processing
            //if mining grid data empty resolve issues to avoid exception
            if (numPointsY == 0 && !gridCreated || numPointsX == 0 && !gridCreated || gridSize == 0 && !gridCreated)
            {
                gridBorePosition.Clear();
                gridBoreOccupied.Clear();
                gridBoreFinished.Clear();
                gridCreated = true;
                gridCentreGPSCoordinates = miningGPSCoordinates;
                gridBoreOccupied.Add(false);
                gridBoreFinished.Add(false);
                gridBorePosition.Add(gridCentreGPSCoordinates);
                totalMiningRuns = gridBorePosition.Count;
                currentGPSIndex = 0;
                if (readyFlag)
                {
                    readyFlag = false;
                }
                if (perimiterInt == 0 && perimeterOnly)
                {
                    perimeterOnly = false;
                }
                if (perimiterInt == 1 && !perimeterOnly)
                {
                    perimeterOnly = true;
                }
            }
            if (!gridCreated)
            {

                if (!bores_regen)
                {
                    gridBorePosition.Clear();
                    gridBoreFinished.Clear();
                    gridBoreOccupied.Clear();
                    if (perimiterInt == 0 && perimeterOnly)
                    {
                        perimeterOnly = false;
                    }
                    if (perimiterInt == 1 && !perimeterOnly)
                    {
                        perimeterOnly = true;
                    }
                    bores_regen = true;
                }

                if (readyFlag)
                {
                    readyFlag = false;
                }

                if (remoteControlTag[0] == null || remoteControlActual == null)
                {
                    sbtexttemp.AppendLine($"Remote control {antennaTagName.Replace("[", "[[").Replace("]", "]]")} not present - early exit");
                    return;
                }
                Vector3D gravity = remoteControlActual.GetNaturalGravity();
                GetCustomDataJobCommand(Me.CustomData, Me);
                if (prospectAlignTargetValid || customDataAlignTargetValid)
                {
                    planeNrml = ((miningGPSCoordinates - alignGPSCoordinates));
                }
                if (!prospectAlignTargetValid && !customDataAlignTargetValid)
                {
                    planeNrml = gravity;
                }

                planeNrml.Normalize();
                Vector3D perpendicularVector = Vector3D.CalculatePerpendicularVector(planeNrml);
                perpendicularVector.Normalize();
                Vector3D centerPoint = miningGPSCoordinates;
                //load from storage if present (test required)

                if (!string.IsNullOrEmpty(Storage) && !string.IsNullOrWhiteSpace(Storage) && !gridCreated && bores_regen && !gridInitialisationComplete && !loadsave)
                {
                    //added from init
                    currentGPSIndex = 0;
                    realGPSIndex = currentGPSIndex;
                    GetStoredData(Storage);
                    sbtexttemp.AppendLine("Grid positions restored");
                    canLoading = true;
                    Storage = null;
                    //reset everything else
                    dronesPinged = false;
                    dronePingTimerCount = 0;
                    gridInitialisationComplete = true;
                }
                //coroutine management grid creation
                if (gridCoroutine == null && !gridInitialisationComplete && bores_regen || gridCoroutine != null && !gridCoroutine.MoveNext() && !gridInitialisationComplete && bores_regen)
                {
                    gridCoroutine = GenGrdPosits(centerPoint, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid, perimeterOnly);
                }
                if (gridCoroutine != null && !gridInitialisationComplete && bores_regen)
                {
                    // Check the current yield value
                    bool currentYield = gridCoroutine.Current;

                    // If the coroutine is finished, you can perform completion logic
                    if (!gridCoroutine.MoveNext())
                    {
                        // The coroutine has finished executing
                        sbtexttemp.AppendLine("Grid generation complete.");
                        gridCoroutine?.Dispose();
                        gridCoroutine = null;
                        GenGrdPosits(centerPoint, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid, perimeterOnly).Dispose();
                    }
                    else
                    {
                        // Handle intermediate status if needed
                        if (!currentYield)
                        {
                            sbtexttemp.AppendLine($"Generating grid positions... {Math.Round(percent_grid, 1)}%");
                            gridCoroutine.MoveNext();

                        }
                        if (currentYield)
                        {
                            //debugcount++;
                            gridInitialisationComplete = true;
                            //clear interface command here
                            writeInterfaceCommand(pbInterfaceActual, "");
                            //pbInterfaceActual.CustomData = "";
                            canInit = false;
                            i_init = false;
                            interfaceArgument = "";
                        }
                    }

                }
                if (!gridCreated && bores_regen && gridInitialisationComplete && loadsave)
                {
                    //added from init
                    gridBorePosition.Clear();
                    gridBoreFinished.Clear();
                    gridBoreOccupied.Clear();
                    currentGPSIndex = 0;
                    realGPSIndex = currentGPSIndex;

                    GetSavedJobData(Me);
                    sbtexttemp.AppendLine("Grid positions restored");
                    canLoading = true;
                    Storage = null;
                    //reset everything else
                    dronesPinged = false;
                    dronePingTimerCount = 0;
                    gridInitialisationComplete = true;
                    loadsave = false;
                }

                //grid data found - terminite initialisation
                if (gridBorePosition.Count > 0 && gridInitialisationComplete)
                {
                    gridCreated = true;
                    //clear interface command here
                    writeInterfaceCommand(pbInterfaceActual, "");
                    //pbInterfaceActual.CustomData = "";
                    canInit = false;
                    i_init = false;
                    interfaceArgument = "";
                }

                totalMiningRuns = gridBorePosition.Count;

                if (numPointsY == 0 || numPointsY == 0 || gridSize == 0 || numPointsY == 0 && numPointsY == 0 && gridSize == 0)
                {
                    miningGridValid = false;
                }
                if (numPointsY > 0 && numPointsY > 0 && gridSize > 0)
                {
                    miningGridValid = true;
                }
                if (!miningGridValid)
                {
                    totalMiningRuns = 1;
                }
                totalMiningSequencesComplete = 0;
                gridBoresCompleted = 0;
                currentGPSIndex = 0;
            }

            if (gridBorePosition.Count > 0)
            {
                if (spriteCountLimit != gridBorePosition.Count + (gridBorePosition.Count / 2) + (Swarm.Count * 10))
                {
                    spriteCountLimit = gridBorePosition.Count + (gridBorePosition.Count / 2) + (Swarm.Count * 10);
                }
                if (spritecount_limit_insert != (gridBorePosition.Count / 2))
                {
                    spritecount_limit_insert = (gridBorePosition.Count / 2);
                }
            }
            else
            {
                if (spriteCountLimit != 500)
                {
                    spriteCountLimit = 500;
                }
                if (spritecount_limit_insert != 250)
                {
                    spritecount_limit_insert = 250;
                }
            }

            sbtexttemp.AppendLine($"Grid: {gridCreated} - Bores: {totalMiningRuns} - Remaining: {boresRemaining}");
            #endregion
        }

        private void ProcessMessages()
        {
            #region check_drone_messages
            //manage recieved communications
            if (antennaActual != null && antennaTag[0] != null)
            {
                if (listenDrones.HasPendingMessage)
                {
                    MyIGCMessage droneMessageNew = listenDrones.AcceptMessage();
                    droneMessagesBuffer.Add(droneMessageNew);
                }
                //process drone message list here
                if (droneMessagesBuffer.Count > 0)
                {
                    droneMessageReceived = true;
                }
                if (droneMessagesBuffer.Count > 0)
                {
                    //pull first message in the list if valid
                    droneDataInput = droneMessagesBuffer[0].Data.ToString();
                    ProcessDroneMessageData(droneDataInput);
                    ProcessReceivedDroneMessageToDroneLists();
                }
                if (droneMessagesBuffer.Count <= 0)
                {
                    droneMessageReceived = false;
                }
                #endregion
                #region check_prospector_messages
                //process drone message list here
                if (listenProspector.HasPendingMessage)
                {
                    MyIGCMessage propsectorMessageNew = listenProspector.AcceptMessage();
                    prospectorMessagesBuffer.Add(propsectorMessageNew);

                }
                //process prospector message list here
                if (prospectorMessagesBuffer.Count <= 0)
                {
                    prospectorMessageReceived = false;
                }
                if (prospectorMessagesBuffer.Count > 0)
                {
                    prospectorMessageReceived = true;
                    prospectorDataInput = prospectorMessagesBuffer[0].Data.ToString();
                    if (remoteControlActual != null && remoteControlTag[0] != null)
                    {
                        StoreRawInput(prospectorDataInput, remoteControlActual, gmdccategory, jobinfo);
                        //remoteControlActual.CustomData = prospectorDataInput;
                    }
                    else
                    {
                        sbtexttemp.AppendLine($"Remote control {antennaTagName.Replace("[", "[[").Replace("]", "]]")} not present");
                        return;
                    }
                    prospectorMessagesBuffer.RemoveAt(0);
                    gridCreated = false;
                    i_init = true;
                }
            }
            #endregion
        }

        private void PingDrones()
        {
            #region drone_lifecheck_ping
            //manage drone ping communications
            if (antennaActual != null && antennaTag[0] != null)
            {
                string syncMessageOut = "";
                if (Swarm.Count == 0 && !dronesPinged || Swarm.Count > 0 && !dronesPinged)
                {
                    IGC.SendBroadcastMessage(txDronePingChannel, pingMessage, TransmissionDistance.TransmissionDistanceMax);
                    if (!string.IsNullOrEmpty(syncMessage) && !string.IsNullOrWhiteSpace(syncMessage))
                    {
                        syncMessageOut = syncMessage;
                    }
                    else
                    {
                        syncMessageOut = "";
                    }
                    IGC.SendBroadcastMessage(txDroneSyncChannel, syncMessageOut, TransmissionDistance.TransmissionDistanceMax);
                    dronesPinged = true;
                    dronePingTimerCount = 0;
                }
            }
            #endregion
        }

        private void ValidateCustomData()
        {
            if (_cachedCustomData != Me.CustomData)
            {
                _cachedCustomData = Me.CustomData;
            }

            if (!string.IsNullOrWhiteSpace(_cachedCustomData))
            {
                mainCustomDataValid = true;
            }
            else
            {
                sbtexttemp.AppendLine($"Job custom data invalid - initialising job data");
                mainCustomDataValid = false;
                if (_cachedCustomData != "GPS:---:0:0:0:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:False")
                {
                    miningCoordinatesNew.Clear();
                    miningCoordinatesNew.Append($"GPS:---:0:0:0:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:False");
                    InvalidJobDataWrite(Me, miningCoordinatesNew.ToString());
                }
            }
            if (mainCustomDataValid)
            {
                if (_cachedCustomData == _oldCustomData)
                {
                    return;
                }
                else if (_cachedCustomData != _oldCustomData)
                {
                    GetCustomDataJobCommand(_cachedCustomData, Me);
                    _oldCustomData = _cachedCustomData;
                }
            }
        }
        void InvalidJobDataWrite(IMyTerminalBlock block, string input)
        {
            _customDataStore.Clear();
            _customDataStore.Set(gmdccategory, jobinfo, input);
            /*_customDataStore.Set(gmdccategory, "TargetGPS", $"GPS:---:0:0:0:#FF75C9F1:");
            _customDataStore.Set(gmdccategory, "AlignGPS", "");
            _customDataStore.Set(gmdccategory, "BoreSeparation", "10.0");
            _customDataStore.Set(gmdccategory, "GridXBores", "1");
            _customDataStore.Set(gmdccategory, "GridYBores", "1");
            _customDataStore.Set(gmdccategory, "SkipBores", "0");
            _customDataStore.Set(gmdccategory, "SafeAlignDistance", "5.0");
            _customDataStore.Set(gmdccategory, "DrillDepth", "5.0");
            _customDataStore.Set(gmdccategory, "IgnoreDepth", "0.0");
            _customDataStore.Set(gmdccategory, "LimitDronesInFlight", "False");
            _customDataStore.Set(gmdccategory, "DronesFlightHardLimit","10");
            _customDataStore.Set(gmdccategory, "DronesFlightFactor", "1");
            _customDataStore.Set(gmdccategory, "CoreOutFunction", "False"); */
            block.CustomData = _customDataStore.ToString();
            _customDataStore.Clear();
        }
        private void InitializeMiningGrid()
        {
            #region initialise_mining_grid_loading
            if (!gridInitialisationComplete && initialisedGridCount > 0)
            {
                initialisedGridCount = 0;
            }
            if (gridInitialisationComplete && initialisedGridCount >= 1 || canInit && gridInitialisationComplete)
            {
                initialisedGridCount = 0;
                gridInitialisationComplete = false;
            }
            if (canInit && gridCreated && !gridInitialisationComplete && !canLoading)
            {
                gridCreated = false;
                miningGridValid = false;
                currentGPSIndex = 0;
                gpsGridPositionValue = -1;
                bores_regen = false;
                gridInitialisationComplete = false;
                prospectAlignTargetValid = false;
                customDataAlignTargetValid = false;
            }
            if (canLoading)
            {
                canLoading = false;
            }
            #endregion
        }

        private void CheckSystemSetupStatus()
        {
            if (!setupComplete)
            {
                SetupSystem();
                setupComplete = true;
                Echo("Setup complete!");
            }
            ComponentPresenceCheck();
            if (!setupComplete)
            {
                return;
            }
        }

        private void ProcessInterface()
        {
            #region Interface_detection
            if (interfacePBTag.Count > 0)
            {
                if (interfacePBTag[0] != null)
                {
                    pbInterfaceActual = interfacePBTag[0];
                }
                else
                {
                    sbtexttemp.AppendLine($"Interface programmable block not present {interface_display}");
                    return;
                }
                canInterfaceCommand = true;
                //read interface command here
                //interfaceArgument = pbInterfaceActual.CustomData;
                if (interfacecommandOld != pbInterfaceActual.CustomData)
                {
                    readInterfaceCommand(pbInterfaceActual);
                    interfacecommandOld = pbInterfaceActual.CustomData;
                }
                sbtexttemp.AppendLine($"Interface PB: {interface_display}");
                sbtexttemp.AppendLine($"Display command: {interfaceArgument} P:{prospectAlignTargetValid} C:{customDataAlignTargetValid}");
            }
            else
            {
                sbtexttemp.AppendLine($"Interface programmable block not present {interface_display}");
                return;
            }
            #endregion
            #region interface_command_processing
            if (interfacePBTag.Count > 0)
            {
                if (canInterfaceCommand && !string.IsNullOrEmpty(pbInterfaceActual.CustomData))
                {
                    if (interfaceArgument == "" && !noInterfaceCommand)
                    {
                        noInterfaceCommand = true;
                    }
                    else
                    {
                        noInterfaceCommand = false;
                    }
                    if (interfaceArgument.Contains("init") && !i_init)
                    {
                        i_init = true;
                    }
                    if (!interfaceArgument.Contains("init") && i_init)
                    {
                        i_init = false;
                    }
                    if (interfaceArgument.Contains("reset") && !i_res)
                    {
                        i_res = true;
                    }
                    else
                    {
                        i_res = false;
                    }
                    if (interfaceArgument.Contains("run") && !i_run)
                    {
                        i_run = true;
                    }
                    else
                    {
                        i_run = false;
                    }
                    if (interfaceArgument.Contains("recall") && !i_recall)
                    {
                        i_recall = true;
                    }
                    else
                    {
                        i_recall = false;
                    }
                    if (interfaceArgument.Contains("eject") && !i_eject)
                    {
                        i_eject = true;
                    }
                    else
                    {
                        i_eject = false;
                    }
                    if (interfaceArgument.Contains("freeze") && !i_frz)
                    {
                        i_frz = true;
                    }
                    else
                    {
                        i_frz = false;
                    }
                    if (interfaceArgument.Contains("stop") && !i_stop)
                    {
                        i_stop = true;
                    }
                    else
                    {
                        i_stop = false;
                    }
                }

                if (!canInterfaceCommand || noInterfaceCommand || pbInterfaceActual.CustomData == null)
                {
                    i_frz = false;
                    i_eject = false;
                    i_recall = false;
                    i_run = false;
                    i_res = false;
                    i_init = false;
                }
            }
            #endregion
        }

        private void HandleCommands(string argument)
        {
            #region run_command_processing
            if (argument == "setup" && setupComplete)
            {
                setupComplete = false;
                argument = "";
                sbtexttemp.AppendLine("Running Setup..");
            }
            if (argument.Contains("run") || i_run)
            {
                canRun = true;
                canReset = false;
                canTransmit = true;
                mustRecall_Command = false;
                canInit = false;
                mustFreeze_Command = false;
                commandAsk = "Run";
            }
            if (argument.Contains("reset") || i_res)
            {
                canReset = true;
                mustUndockCommand = false;
                canRun = false;
                canTransmit = true;
                mustRecall_Command = false;
                canInit = false;
                mustFreeze_Command = false;
                commandAsk = "Reset";
                currentGPSIndex = 0;
            }
            if (argument.Contains("stop") || i_stop)
            {
                canTransmit = false;
                mustUndockCommand = false;
                canRun = false;
                canReset = false;
                mustRecall_Command = false;
                canInit = false;
                mustFreeze_Command = false;
                commandAsk = "Stop";
            }
            if (argument.Contains("recall") || i_recall)
            {
                mustRecall_Command = true;
                mustUndockCommand = false;
                canReset = false;
                canTransmit = true;
                canRun = false;
                mustFreeze_Command = false;
                commandAsk = "Recall";
                currentGPSIndex = 0;
            }
            if (argument.Contains("init") || i_init)
            {
                mustRecall_Command = false;
                mustUndockCommand = false;
                canReset = false;
                canTransmit = false;
                canRun = false;
                canInit = true;
                mustFreeze_Command = false;
                commandAsk = "Init";
                currentGPSIndex = 0;
                realGPSIndex = currentGPSIndex;
                Storage = null;

            }
            if (argument.Contains("eject") || i_eject)
            {
                mustRecall_Command = false;
                mustUndockCommand = true;
                canReset = false;
                canTransmit = true;
                canRun = false;
                mustFreeze_Command = false;
                commandAsk = "Eject";
                currentGPSIndex = 0;
            }
            if (argument.Contains("freeze") || i_frz)
            {
                canTransmit = true;
                mustFreeze_Command = true;
                mustUndockCommand = false;
                canRun = false;
                canReset = false;
                mustRecall_Command = false;
                canInit = false;
                commandAsk = "Freeze";
            }
            if (mustUndockCommand || mustRecall_Command || mustFreeze_Command)
            {
                disableRunArgument = true;
            }
            else
            {
                disableRunArgument = false;
            }
            #endregion
        }

        private void checkDroneID()
        {
            if (droneID.Count != Swarm.Count)
            {
                droneID.Clear();
                foreach (DroneData drone in Swarm.Values)
                {
                    droneID.Add(drone.Name);
                }
            }
        }
        private void DroneRenderCall(bool sort = false)
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (display_tag_drone.Count == 0 || Swarm.Count == 0 || myTextSurfaces_d2.Count == 0) return;
            checkDroneID();

            if (renew_header)
            {
                droneInformation.Clear().Append($"Mining Drone Status {secondary_tag} [{drone_tag}] - GMDC {ver} {icon}\n");
                renew_header = false;
            }


            int dronesMaxDisplay = drones_per_screen * myTextSurfaces_d2.Count;
            if (dronesMaxDisplay < droneID.Count)
            {
                sbtexttemp.AppendLine($"Insufficient displays '{dp_drn_tag.Replace("[", "[[").Replace("]", "]]")}': {dronesMaxDisplay} < {droneID.Count}");
                return;
            }
            if (sort)
            {
                droneID.Sort(); //Optional Sorting
            }
            if ((drones_per_screen > 0 && drones_per_screen <= 4))
            {
                for (int i = 0; i < droneID.Count; i++)
                {
                    bool hasPair = false;
                    DroneScreenBuilder(i, hasPair ? i + 1 : i, hasPair);

                    int displayIndex = i / drones_per_screen;
                    if (displayIndex < myTextSurfaces_d2.Count && myTextSurfaces_d2[displayIndex] != null)
                    {
                        myTextSurfaces_d2[displayIndex].WriteText(droneInformation);
                        renew_header = true;
                    }
                }
            }
            else
            {
                for (int i = 0; i < droneID.Count; i += 2)
                {
                    bool hasPair = i + 1 < droneID.Count;
                    DroneScreenBuilder(i, hasPair ? i + 1 : i, hasPair);

                    int displayIndex = i / drones_per_screen;
                    if (displayIndex < myTextSurfaces_d2.Count && myTextSurfaces_d2[displayIndex] != null &&
                        (i % drones_per_screen == drones_per_screen - 2 || i >= droneID.Count - 2))
                    {
                        myTextSurfaces_d2[displayIndex].WriteText(droneInformation);
                        renew_header = true;
                    }
                }
            }

            //sbtexttemp.AppendLine($"drone_render_call: {Runtime.CurrentInstructionCount - startInstructions}");
        }
        private struct DroneStats
        {
            public int Docking, Docked, Undocking, Undocked, Damage, Unknown, Ok, Exit, Idle, Recharge, Unload, Mining, RTBA, RTBB, Nav, IdleD;
        }

        private void UpdateDroneCounts()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            gridBoresCompleted = CountTrueValuesList(gridBoreFinished);
            boresRemaining = totalMiningRuns - gridBoresCompleted;
            totalDronesActive = CountTrueValues("IsMining");
            total_drones_undocking = CountIntegerValues("ControlSequence", 2);

            DroneStats stats = new DroneStats();
            foreach (DroneData drone in Swarm.Values)
            {
                string status = drone.ControlStatus;
                string damage = drone.DamageState;
                stats.Docking += status.Contains("Docking") ? 1 : 0;
                stats.Docked += status.Contains("Docked") ? 1 : 0;
                stats.Undocking += status.Contains("Undocking") ? 1 : 0;
                stats.Undocked += status.Contains("Undocked") ? 1 : 0;
                stats.Exit += status.Contains("Exit") ? 1 : 0;
                stats.Idle += status.Equals("Idle") ? 1 : 0;
                stats.Recharge += status.Contains("Recharg") ? 1 : 0;
                stats.Unload += status.Contains("Unload") ? 1 : 0;
                stats.Mining += status.Contains("Min") ? 1 : 0;
                stats.RTBA += status.Contains("RTB: Request") ? 1 : 0;
                stats.RTBB += status.Contains("RTB: Ready") ? 1 : 0;
                stats.Nav += status.Contains("Nav") ? 1 : 0;
                stats.IdleD += status.Equals("Docked Idle") ? 1 : 0;
                stats.Damage += damage == "DMG" ? 1 : 0;
                stats.Unknown += damage == "UNK" ? 1 : 0;
                stats.Ok += damage == "OK" ? 1 : 0;
            }
            t_drn_dckg = stats.Docking; t_drn_dck = stats.Docked; t_drn_udckg = stats.Undocking;
            t_drn_udck = stats.Undocked; t_drn_exit = stats.Exit; t_drn_idle_undocked = stats.Idle;
            t_drn_rechg = stats.Recharge; t_drn_unload = stats.Unload; t_drn_mine = stats.Mining;
            t_drn_nav = stats.Nav; t_drn_idle_docked = stats.IdleD;
            totalDronesDamaged = stats.Damage; totalDronesUnknown = stats.Unknown; // t_dn_ok = stats.Ok;
            totalDronesMining = totalDronesActive - t_drn_dckg;
            if (totalDronesMining < 0)
            {
                totalDronesMining = 0;
            }
            //sbtexttemp.AppendLine($"UpdateDroneCounts: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private int[] boreQueueCounts = new int[0]; // Static reuse

        private void DroneUndockCheck()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (total_drones_undocking > 0)
                dronesUndocking = true;
            if (dronesUndocking)
            {
                undockTimer++;
                if (undockTimer >= undock_delay_limit)
                {
                    dronesUndocking = false;
                    undockTimer = 0;
                }
            }
            else if (undockTimer != 0)
                undockTimer = 0;

            if (gridBoreOccupied.Count > 0)
            {
                if (boreQueueCounts.Length < gridBoreOccupied.Count)
                    boreQueueCounts = new int[gridBoreOccupied.Count];
                Array.Clear(boreQueueCounts, 0, gridBoreOccupied.Count);

                foreach (DroneData drone in Swarm.Values)
                    if (drone.GpsListPosition >= 0 && drone.GpsListPosition < boreQueueCounts.Length)
                        boreQueueCounts[drone.GpsListPosition]++;


                for (int l = 0; l < gridBoreOccupied.Count; l++)
                    if (gridBoreOccupied[l] && boreQueueCounts[l] == 0)
                        gridBoreOccupied[l] = false;
            }
            //sbtexttemp.AppendLine($"DroneUndockCheck: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        public void updateDisplay(DroneData drone)
        {
            displayTextMain.Append($"Drone Controller Status - GMDC {ver} - [{drone_tag}] {icon}");
            displayTextMain.Append('\n');
            if (drone.ControlSequence == 12)
            {
                gpsGridPositionValue = -1;
                drone.IsMining = false;
                displayTextMain.Append('\n');
                displayTextMain.Append("Mining seq. complete"); ;
            }

            if (gridBorePosition.Count > 0)
            {
                displayTextMain.Append('\n');
                displayTextMain.Append("Grid pos: " + gridBorePosition.Count);
                displayTextMain.Append('\n');
                displayTextMain.Append("Grid dist: " + customData7 + "m #X: " + customData8 + " #Y: " + customData9);
                displayTextMain.Append('\n');
                displayTextMain.Append("Grid OK: " + miningGridValid);
                displayTextMain.Append('\n');
                displayTextMain.Append("Bores: " + gridBorePosition.Count + " Remain: " + boresRemaining + "  Skip: " + skipBoresNumber);
                displayTextMain.Append('\n');
            }
            else
            {
                displayTextMain.Append('\n');
                displayTextMain.Append("Bores: " + totalMiningRuns + " Remaining: " + boresRemaining);
            }
            if (totalMiningRuns > 0)
            {
                displayTextMain.Append('\n');
                displayTextMain.Append("Current mine idx: " + realGPSIndex + " of " + (totalMiningRuns - 1) + " (" + gridBoresCompleted + ") ");
            }
            else
            {
                displayTextMain.Append('\n');
                displayTextMain.Append("Current mine idx: " + currentGPSIndex + " of " + (totalMiningRuns - 1) + " (" + gridBoresCompleted + ") ");
            }
            if (currentGPSIndex > totalMiningRuns || !miningGridValid && gridBoresCompleted >= totalMiningRuns || boresRemaining == 0)
            {
                canRun = false;
                displayTextMain.Append('\n');
                displayTextMain.Append("Mine seq. complete");
                displayTextMain.Append('\n');
                drone.ControlSequence = 12;
                currentGPSIndex = 0;
            }
        }
        void ProcessDroneMessageData(string dataMessageInput)
        {
            // get custom data from programmable block
            String[] messageData = dataMessageInput.Split(':');

            //Define GPS coordinates from 
            if (messageData.Length > 5)
            {
                receivedDroneName = messageData[0];
                if (receivedDroneName.Contains(drone_tag))
                {
                    receivedDroneDamageStatus = messageData[1];
                    receivedDroneTunnelFinished = messageData[2];
                    receivedDroneStatus = messageData[3];
                    receivedDroneDocked = messageData[4];
                    receivedDroneUndocked = messageData[5];
                    recived_drone_autopilot = messageData[6];
                    rc_auto_pilot_enabled = messageData[7];
                    rc_locx = messageData[8];
                    rc_locy = messageData[9];
                    rc_locz = messageData[10];
                    rc_dn_drl_dpth = messageData[11];
                    rc_dn_drl_crnt = messageData[12];
                    rc_dn_drl_strt = messageData[13];
                    rc_dn_chg = messageData[14];
                    rc_dn_gas = messageData[15];
                    rc_dn_str = messageData[16];
                    if (messageData.Length > 16)
                    {
                        rc_dn_gps_lst = messageData[17];
                    }
                    if (messageData.Length > 17)
                    {
                        rc_dn_cargo_full = messageData[18];
                    }
                    if (messageData.Length > 18)
                    {
                        rc_dn_rchg_req = messageData[19];
                    }
                    if (messageData.Length > 19)
                    {
                        recievedDroneAutdock = messageData[20];
                    }
                    if (messageData.Length > 20)
                    {
                        recievedDroneDockingReady = messageData[21];
                    }
                }
                else
                {
                    receivedDroneName = "";
                    receivedDroneDamageStatus = "";
                    receivedDroneTunnelFinished = "";
                    receivedDroneStatus = "";
                    receivedDroneDocked = "";
                    receivedDroneUndocked = "";
                    recived_drone_autopilot = "";
                    rc_auto_pilot_enabled = "";
                    rc_locx = "";
                    rc_locy = "";
                    rc_locz = "";
                    rc_dn_drl_dpth = "";
                    rc_dn_drl_crnt = "";
                    rc_dn_drl_strt = "";
                    rc_dn_chg = "";
                    rc_dn_gas = "";
                    rc_dn_str = "";
                    rc_dn_gps_lst = "";
                    rc_dn_cargo_full = "";
                    rc_dn_rchg_req = "";

                }
                if (rc_dn_gps_lst == "")
                {
                    recieved_drone_list_position = -1;
                }
                else
                {
                    if (!int.TryParse(rc_dn_gps_lst, out recieved_drone_list_position))
                    {
                        recieved_drone_list_position = -1;
                    }
                }
                if (rc_dn_chg == "")
                {
                    rc_d_cn = 0.0;
                }
                if (!double.TryParse(rc_dn_chg, out rc_d_cn))
                {
                    rc_d_cn = 0.0;
                }
            }
        }
        void GetRemoteControlData(string input, IMyTerminalBlock block)
        {
            if (block == null || remoteControlTag[0] == null)
            {
                Echo($"Remote Control {antennaTagName.Replace("[", "[[").Replace("]", "]]")} not present");
                return;
            }
            if (string.IsNullOrEmpty(block.CustomData) || string.IsNullOrWhiteSpace(block.CustomData))
            {
                sbtexttemp.AppendLine("Prospector job data not found");
                return;
            }
            // Checks if the block has CustomData AND if it's NOT already INI-formatted data
            if (!string.IsNullOrEmpty(block.CustomData) && !block.CustomData.Contains("[GMDCJobData]"))
            {
                String[] remoteGpsCommandtest = block.CustomData.Split(':');

                if (remoteGpsCommandtest.Length > 0)
                {
                    StoreRawInput(block.CustomData, block, gmdccategory, rcjobinfo);

                }
                return;

            }
            if (string.IsNullOrWhiteSpace(input) || string.IsNullOrEmpty(input))
            {
                return;
            }

            FetchRCJobData(remoteControlActual);
            String[] remoteGpsCommand = rcjobdata.Split(':');

            if (remoteGpsCommand.Length < 6)
            {
                //  remoteControlCustomData1 = "";
                remoteControlCustomData2 = "";
                remoteControlCustomData3 = "";
                remoteControlCustomData4 = "";
                //   remoteControlCustomData5 = "";
                remoteControlCustomData6 = "";

                prospectTargetValid = false;
                return;
            }
            if (remoteGpsCommand.Length > 6)
            {

                //target_gps_coords = new Vector3D(Double.Parse(remoteGpsCommand[2]), Double.Parse(remoteGpsCommand[3]), Double.Parse(remoteGpsCommand[4]));
                prospectTargetValid = true;
                //  remoteControlCustomData1 = remoteGpsCommand[1];
                remoteControlCustomData2 = remoteGpsCommand[2];
                remoteControlCustomData3 = remoteGpsCommand[3];
                remoteControlCustomData4 = remoteGpsCommand[4];
                //  remoteControlCustomData5 = remoteGpsCommand[5];
                remoteControlCustomData6 = remoteGpsCommand[6];
                if (!double.TryParse(remoteControlCustomData2, out targetGPSCoordinates.X))
                {
                    targetGPSCoordinates.X = 0.0;
                    remoteControlCustomData2 = "";
                }
                if (!double.TryParse(remoteControlCustomData3, out targetGPSCoordinates.Y))
                {
                    targetGPSCoordinates.Y = 0.0;
                    remoteControlCustomData3 = "";
                }
                if (!double.TryParse(remoteControlCustomData4, out targetGPSCoordinates.Z))
                {
                    targetGPSCoordinates.Z = 0.0;
                    remoteControlCustomData4 = "";
                }
                //5 is colour data
                if (!double.TryParse(remoteControlCustomData6, out safe_dstvl))
                {
                    safe_dstvl = 0.0;
                }

            }
            if (remoteGpsCommand.Length < 11 && remoteGpsCommand.Length > 7 && !prospectAlignTargetValid)
            {
                //  remoteControlCustomData7 = "";
                //   remoteControlCustomData8 = "";
                remoteControlCustomData9 = "";
                remoteControlCustomData10 = "";
                remoteControlCustomData11 = "";
                //   remoteControlCustomData12 = "";
                prospectAlignTargetValid = false;
                return;
            }
            if (remoteGpsCommand.Length > 7 && !prospectAlignTargetValid)
            {
                bool AlignX = false;
                bool AlignY = false;
                bool AlignZ = false;

                //   remoteControlCustomData7 = remoteGpsCommand[7];
                //   remoteControlCustomData8 = remoteGpsCommand[8];
                remoteControlCustomData9 = remoteGpsCommand[9];
                remoteControlCustomData10 = remoteGpsCommand[10];
                remoteControlCustomData11 = remoteGpsCommand[11];
                //   remoteControlCustomData12 = remoteGpsCommand[12];
                if (!double.TryParse(remoteControlCustomData9, out alignGPSCoordinates.X))
                {
                    alignGPSCoordinates.X = 0.0;
                    remoteControlCustomData9 = "";
                    AlignX = false;
                }
                else
                {
                    AlignX = true;
                }
                if (!double.TryParse(remoteControlCustomData10, out alignGPSCoordinates.Y))
                {
                    alignGPSCoordinates.Y = 0.0;
                    remoteControlCustomData10 = "";
                    AlignY = false;
                }
                else
                {
                    AlignY = true;
                }
                if (!double.TryParse(remoteControlCustomData11, out alignGPSCoordinates.Z))
                {
                    alignGPSCoordinates.Z = 0.0;
                    remoteControlCustomData11 = "";
                    AlignZ = false;
                }
                else
                {
                    AlignZ = true;
                }
                if (AlignX && AlignY && AlignZ)
                {
                    prospectAlignTargetValid = true;
                }

                StoreRCJobData(remoteControlActual, rcjobdata);
            }
        }

        void FetchRCJobData(IMyTerminalBlock input)
        {
            var str = "";
            bool rcx = false;
            bool rcy = false;
            bool rcz = false;
            _customDataStore.Clear();
            if (_customDataStore.TryParse(input.CustomData.ToString()))
            {
                str = _customDataStore.Get(gmdccategory, rcjobinfo).ToString().Trim();
                rcjobdata = str;
                str = _customDataStore.Get(gmdccategory, "TargetGPS").ToString().Trim();
                String[] vectorsplit = str.Split(':');
                if (vectorsplit.Length >= 5)
                {
                    if (!double.TryParse(vectorsplit[2], out targetGPSCoordinates.X))
                    {
                        targetGPSCoordinates.X = 0.0;
                    }
                    if (!double.TryParse(vectorsplit[3], out targetGPSCoordinates.Y))
                    {
                        targetGPSCoordinates.Y = 0.0;
                    }
                    if (!double.TryParse(vectorsplit[4], out targetGPSCoordinates.Z))
                    {
                        targetGPSCoordinates.Z = 0.0;
                    }
                }
                else
                {
                    targetGPSCoordinates.X = 0.0;
                    targetGPSCoordinates.Y = 0.0;
                    targetGPSCoordinates.Z = 0.0;
                }
                str = _customDataStore.Get(gmdccategory, "AlignGPS").ToString().Trim();
                String[] vectorsplita = str.Split(':');
                if (vectorsplita.Length >= 5)
                {
                    if (!double.TryParse(vectorsplita[2], out alignGPSCoordinates.X))
                    {
                        alignGPSCoordinates.X = 0.0;
                    }
                    else
                    {
                        rcx = true;
                    }
                    if (!double.TryParse(vectorsplita[3], out alignGPSCoordinates.Y))
                    {
                        alignGPSCoordinates.Y = 0.0;
                    }
                    else
                    {
                        rcy = true;
                    }
                    if (!double.TryParse(vectorsplita[4], out alignGPSCoordinates.Z))
                    {
                        alignGPSCoordinates.Z = 0.0;
                    }
                    else
                    {
                        rcz = true;
                    }
                }
                else
                {
                    alignGPSCoordinates.X = 0.0;
                    alignGPSCoordinates.Y = 0.0;
                    alignGPSCoordinates.Z = 0.0;
                }

                str = _customDataStore.Get(gmdccategory, "SafeAlignDistance").ToString().Trim();
                if (!double.TryParse(str, out safe_dstvl))
                {
                    safe_dstvl = 30.0;
                }
            }
            if (rcx && rcy && rcz && !customDataAlignTargetValid)
            {
                customDataAlignTargetValid = true;
            }
            _customDataStore.Clear();
        }
        void StoreRCJobData(IMyTerminalBlock block, string input)
        {
            _customDataStore.Clear();
            if (_customDataStore.TryParse(block.CustomData.ToString()))
            {
                _customDataStore.Set(gmdccategory, rcjobinfo, input);
                _customDataStore.Set(gmdccategory, "TargetGPS", $"GPS:PDT:{targetGPSCoordinates.X}:{targetGPSCoordinates.Y}:{targetGPSCoordinates.Z}:#FF75C9F1:");
                _customDataStore.Set(gmdccategory, "AlignGPS", $"GPS:TGT:{alignGPSCoordinates.X}:{alignGPSCoordinates.Y}:{alignGPSCoordinates.Z}:#F77668:");
                _customDataStore.Set(gmdccategory, "SafeAlignDistance", safe_dstvl);
                block.CustomData = _customDataStore.ToString();
            }
            else
            {
                _customDataStore.Set(gmdccategory, rcjobinfo, input);
                _customDataStore.Set(gmdccategory, "TargetGPS", $"GPS:PDT:{targetGPSCoordinates.X}:{targetGPSCoordinates.Y}:{targetGPSCoordinates.Z}:#FF75C9F1:");
                _customDataStore.Set(gmdccategory, "AlignGPS", $"GPS:TGT:{alignGPSCoordinates.X}:{alignGPSCoordinates.Y}:{alignGPSCoordinates.Z}:#F77668:");
                _customDataStore.Set(gmdccategory, "SafeAlignDistance", safe_dstvl);
                block.CustomData = _customDataStore.ToString();
            }
            _customDataStore.Clear();
        }


        void GetCustomDataJobCommand(string input, IMyTerminalBlock block)
        {
            // Checks if the block has CustomData AND if it's NOT already INI-formatted data
            if (!string.IsNullOrEmpty(block.CustomData) && !block.CustomData.Contains(gmdccategory))
            {
                String[] gpsCommandtest = block.CustomData.ToString().Split(':');

                if (gpsCommandtest.Length > 0)
                {
                    StoreRawInput(block.CustomData, block, gmdccategory, jobinfo);
                }
                sbtexttemp.AppendLine("Dataconversion");
                return;

            }
            if (string.IsNullOrWhiteSpace(block.CustomData.ToString()))
            {
                sbtexttemp.AppendLine("Datablank");
                return;
            }
            FetchJobData(block);
            String[] gpsCommand = jobdata.Split(':');

            customDataAlignTargetValid = false;
            if (gpsCommand.Length < 10)
            {
                //customData1 = "";
                customData2 = "";
                customData3 = "";
                customData4 = "";
                customData5 = "";
                customData6 = "";
                customData7 = "";
                customData8 = "";
                customData9 = "";
                customData10 = "";
                customData11 = "";
                customData12 = "";
                customData13 = "";
                customData14 = "";
                customData15 = "";
                // customData16 = "";
                // customData17 = "";
                customData18 = "";
                customData19 = "";
                customData20 = "";
                //  customData21 = "";
                customData22 = "";
                //customData23 = "";
                sbtexttemp.AppendLine("Data format invalid - GPS:name:x:y:z:depth:grid:numx:numy:limit=True/False:flightfactor:flighthardlimit:perimeteronly 0,1");
                return;
            }
            if (gpsCommand.Length > 4)
            {
                bool mAlignX;
                bool mAlignY;
                bool mAlignZ;
                //customData1 = gpsCommand[1];
                customData2 = gpsCommand[2];
                customData3 = gpsCommand[3];
                customData4 = gpsCommand[4];
                customData5 = gpsCommand[5];
                if (!double.TryParse(customData2, out miningGPSCoordinates.X))
                {
                    miningGPSCoordinates.X = 0.0;
                    customData2 = "";
                    mAlignX = false;
                }
                else
                {
                    mAlignX = true;
                }
                if (!double.TryParse(customData3, out miningGPSCoordinates.Y))
                {
                    miningGPSCoordinates.Y = 0.0;
                    customData3 = "";
                    mAlignY = false;
                }
                else
                {
                    mAlignY = true;
                }
                if (!double.TryParse(customData4, out miningGPSCoordinates.Z))
                {
                    miningGPSCoordinates.Z = 0.0;
                    customData4 = "";
                    mAlignZ = false;
                }
                else
                {
                    mAlignZ = true;
                }
                if (mAlignX && mAlignY && mAlignZ)
                {
                    miningCoordsValid = true;
                }
                else
                {
                    miningCoordsValid = false;
                }

            }
            //5 should be colour data
            if (gpsCommand.Length > 6)
            {
                customData6 = gpsCommand[6];
                if (!Double.TryParse(customData6, out drillLength))
                {
                    drillLength = 1.0;
                    customData6 = "";
                }
            }
            if (gpsCommand.Length > 7)
            {
                customData7 = gpsCommand[7];
                if (!Double.TryParse(customData7, out gridSize))
                {
                    gridSize = 0.0;
                    customData7 = "";
                }
            }
            if (gpsCommand.Length > 8)
            {
                customData8 = gpsCommand[8];
                if (!int.TryParse(customData8, out numPointsX))
                {
                    numPointsX = 0;
                    customData8 = "";
                }
            }
            if (gpsCommand.Length > 9)
            {
                customData9 = gpsCommand[9];

                if (!int.TryParse(customData9, out numPointsY))
                {
                    numPointsY = 0;
                    customData9 = "";
                }
            }
            if (gpsCommand.Length > 10)
            {
                customData10 = gpsCommand[10];
                if (!Double.TryParse(customData10, out ignoreDepth))
                {
                    ignoreDepth = 0.0;
                    customData10 = "";
                }
            }
            if (gpsCommand.Length > 11)
            {
                customData11 = gpsCommand[11];
                if (!bool.TryParse(customData11, out dronesLaunchedStatus))
                {
                    dronesLaunchedStatus = false;
                    customData11 = "";
                }
            }
            if (gpsCommand.Length > 12)
            {
                customData12 = gpsCommand[12];
                if (!int.TryParse(customData12, out dronesInFlightFactor))
                {
                    dronesInFlightFactor = 1;
                    customData12 = "";
                }
            }
            if (gpsCommand.Length > 13)
            {
                customData13 = gpsCommand[13];
                if (!int.TryParse(customData13, out dronesActiveHardLimit))
                {
                    dronesActiveHardLimit = 6;
                    customData13 = "";
                }
            }
            if (gpsCommand.Length > 14)
            {
                customData14 = gpsCommand[14];
                if (!int.TryParse(customData14, out perimiterInt))
                {
                    skipBoresNumber = 0;
                    perimiterInt = 0;
                    perimeterOnly = false;
                    customData14 = "";
                }
            }
            if (perimiterInt == 0 && perimeterOnly)
            {
                perimeterOnly = false;
            }
            if (perimiterInt == 1 && !perimeterOnly)
            {
                perimeterOnly = true;
            }
            if (gpsCommand.Length > 15)
            {
                customData15 = gpsCommand[15];
                if (!bool.TryParse(customData15, out coreOutGrid))
                {
                    coreOutGrid = false;
                    customData15 = "";
                }
            }

            if (gpsCommand.Length > 16 && !prospectAlignTargetValid)
            {
                sbtexttemp.AppendLine($"gpsCommandLen:{gpsCommand.Length}");
                bool targetAlignX;
                bool targetAlignY;
                bool targetAlignZ;
                //   if (gpsCommand.Length > 16)
                //   {
                // customData16 = gpsCommand[16];
                //   }
                //  if (gpsCommand.Length > 17)
                //  {
                //       customData17 = gpsCommand[17];
                //   }
                if (gpsCommand.Length > 18)
                {
                    customData18 = gpsCommand[18];
                }
                if (gpsCommand.Length > 19)
                {
                    customData19 = gpsCommand[19];
                }
                if (gpsCommand.Length > 20)
                {
                    customData20 = gpsCommand[20];
                }
                //   if (gpsCommand.Length > 21)
                //   {
                //       customData21 = gpsCommand[21];
                //   }
                if (gpsCommand.Length > 22)
                {
                    customData22 = gpsCommand[22];
                }


                if (!double.TryParse(customData18, out alignGPSCoordinates.X))
                {
                    alignGPSCoordinates.X = 0.0;
                    customData18 = "";
                    targetAlignX = false;
                }
                else
                {
                    targetAlignX = true;
                }
                if (!double.TryParse(customData19, out alignGPSCoordinates.Y))
                {
                    alignGPSCoordinates.Y = 0.0;
                    customData19 = "";
                    targetAlignY = false;
                }
                else
                {
                    targetAlignY = true;
                }
                if (!double.TryParse(customData20, out alignGPSCoordinates.Z))
                {
                    alignGPSCoordinates.Z = 0.0;
                    customData20 = "";
                    targetAlignZ = false;
                }
                else
                {
                    targetAlignZ = true;
                }
                if (targetAlignX && targetAlignY && targetAlignZ)
                {
                    //prospectAlignTargetValid = true;
                    customDataAlignTargetValid = true;
                }
                else
                {
                    prospectAlignTargetValid = false;
                    customDataAlignTargetValid = false;
                }
                if (!double.TryParse(customData22, out safe_dstvl))
                {
                    safe_dstvl = 30.0;
                    customData22 = "";
                }
            }

            if ((prospectAlignTargetValid || customDataAlignTargetValid) && gpsCommand.Length > 16 && gpsCommand.Length < 18)
            {
                string tempbro = jobdata;
                string updater = tempbro + $"GPS:TGT:{alignGPSCoordinates.X}:{alignGPSCoordinates.Y}:{alignGPSCoordinates.Z}:#F77668:{safe_dstvl}:";
                StoreRawInput(updater, block, gmdccategory, jobinfo);
                //StoreJobData(block, updater);
            }

        }
        void FetchJobData(IMyTerminalBlock input)
        {
            var str = "";
            bool _isMissing = false;
            _customDataStore.Clear();
            if (_customDataStore.TryParse(input.CustomData.ToString()))
            {
                if (_customDataStore.ContainsKey(gmdccategory, jobinfo))
                {
                    str = _customDataStore.Get(gmdccategory, jobinfo).ToString().Trim();
                    jobdata = str;

                }
                else
                {
                    //write custom data here to input
                    _customDataStore.Set(gmdccategory, jobinfo, jobdata);

                    _isMissing = true;
                }
                //manage flag for loading data from interface
                if (_customDataStore.ContainsKey("GMDCJobData", "loadsave"))
                {
                    str = _customDataStore.Get("GMDCJobData", "loadsave").ToString().Trim();
                    if (!bool.TryParse(str, out loadsave))
                    {
                        loadsave = false;
                    }
                }
                else
                {
                    //write custom data here to input
                    _customDataStore.Set("GMDCJobData", "loadsave", "false");
                    _isMissing = true;
                }
                //manage jobname
                if (_customDataStore.ContainsKey("GMDCJobData", "jobname"))
                {
                    str = _customDataStore.Get("GMDCJobData", "jobname").ToString().Trim();
                    jobname = str;
                }
                else
                {
                    //write custom data here to input
                    _customDataStore.Set("GMDCJobData", "jobname", "");
                    _isMissing = true;
                }
            }
            if (_isMissing)
            {
                input.CustomData = _customDataStore.ToString();
                _isMissing = false;
            }
            _customDataStore.Clear();
        }
        public void DroneScreenBuilder(int ivl, int ivl2, bool slu)
        {
            if (droneID.Count <= 0) return;

            DroneData drone1;
            DroneData drone2 = null;
            // EXCEPTION & NRE FIX: Safe Dictionary Lookups without allocating 'new DroneData()'
            if (!Swarm.TryGetValue(droneID[ivl], out drone1)) return;

            if (slu)
            {
                if (!Swarm.TryGetValue(droneID[ivl2], out drone2))
                {
                    slu = false; // Fallback safely to single-column if the 2nd drone is missing
                }
            }

            bool d1BoreValid = drone1.GpsListPosition != -1 && gridBoreFinished.Count > 0 && drone1.GpsListPosition < gridBoreFinished.Count;
            bool d2BoreValid = slu && drone2.GpsListPosition != -1 && gridBoreFinished.Count > 0 && drone2.GpsListPosition < gridBoreFinished.Count;

            droneInformation.AppendLine();

            // We use 's' to track the starting length of the builder for dynamic space padding
            int s;

            // Row 0
            s = droneInformation.Length;
            droneInformation.Append(droneID[ivl]).Append(" Status: ").Append(drone1.DamageState).Append(" ").Append(drone1.ControlStatus);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append(droneID[ivl2]).Append(" Status: ").Append(drone2.DamageState).Append(" ").Append(drone2.ControlStatus);
            droneInformation.AppendLine();

            // Row 1
            s = droneInformation.Length;
            droneInformation.Append(droneID[ivl]).Append(" Docked: ").Append(drone1.Docked).Append(" Rdy: ").Append(drone1.IsReady);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append(droneID[ivl2]).Append(" Docked: ").Append(drone2.Docked).Append(" Rdy: ").Append(drone2.IsReady);
            droneInformation.AppendLine();

            // Row 2
            s = droneInformation.Length;
            droneInformation.Append(droneID[ivl]).Append(" Undocked: ").Append(drone1.Undocked).Append(" Gates: ").Append(drone1.AssignedGates.Count).Append(" ");
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append(droneID[ivl2]).Append(" Undocked: ").Append(drone2.Undocked).Append(" Gates: ").Append(drone2.AssignedGates.Count).Append(" ");
            droneInformation.AppendLine();

            // Row 3 (Bore validation)
            s = droneInformation.Length;
            droneInformation.Append(droneID[ivl]).Append(" Finished: ").Append(drone1.TunnelFinished).Append(" Bore: ");
            if (d1BoreValid) droneInformation.Append(gridBoreFinished[drone1.GpsListPosition]);
            else droneInformation.Append("N/A");

            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));

            if (slu)
            {
                droneInformation.Append(droneID[ivl2]).Append(" Finished: ").Append(drone2.TunnelFinished).Append(" Bore: ");
                if (d2BoreValid) droneInformation.Append(gridBoreFinished[drone2.GpsListPosition]);
                else droneInformation.Append("N/A");
            }
            droneInformation.AppendLine();

            // Row 4
            s = droneInformation.Length;
            droneInformation.Append(droneID[ivl]).Append(" Mining: ").Append(drone1.IsMining);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append(droneID[ivl2]).Append(" Mining: ").Append(drone2.IsMining);
            droneInformation.AppendLine();

            // Row 5
            s = droneInformation.Length;
            droneInformation.Append(droneID[ivl]).Append(" Waiting: ").Append(drone1.MustWait).Append(" Reset: ").Append(drone1.ResetFunction);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append(droneID[ivl2]).Append(" Waiting: ").Append(drone2.MustWait).Append(" Reset: ").Append(drone2.ResetFunction);
            droneInformation.AppendLine();

            // Row 6
            s = droneInformation.Length;
            droneInformation.Append("Charge: ").Append(drone1.ChargeStorage).Append("% Tank: ").Append(drone1.GasStorage).Append("% Cargo: ").Append(drone1.OreStorage).Append("%");
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append("Charge: ").Append(drone2.ChargeStorage).Append("% Tank: ").Append(drone2.GasStorage).Append("% Cargo: ").Append(drone2.OreStorage).Append("%");
            droneInformation.AppendLine();

            // Row 7
            s = droneInformation.Length;
            droneInformation.Append("Drill depth: ").Append(drone1.BoreDepth).Append("m Start: ").Append(drone1.MineDepthStartStatus).Append("m");
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append("Drill depth: ").Append(drone2.BoreDepth).Append("m Start: ").Append(drone2.MineDepthStartStatus).Append("m");
            droneInformation.AppendLine();

            // Row 8
            s = droneInformation.Length;
            droneInformation.Append("Current depth: ").Append(drone1.BoreDepthCurrent).Append("m");
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append("Current depth: ").Append(drone2.BoreDepthCurrent).Append("m");
            droneInformation.AppendLine();

            // Row 9
            s = droneInformation.Length;
            droneInformation.Append("Drone control seq: ").Append(drone1.ControlSequence).Append(" Recall seq: ").Append(drone1.RecallSequence).Append(" ").Append(drone1.RecallList);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append("Drone control seq: ").Append(drone2.ControlSequence).Append(" Recall seq: ").Append(drone2.RecallSequence).Append(" ").Append(drone2.RecallList);
            droneInformation.AppendLine();

            // Row 10
            s = droneInformation.Length;
            droneInformation.Append("Location: ").Append(drone1.GpsListPosition).Append(" Asnd: ").Append(drone1.AssignedCoordinates).Append(" Unit OK: ").Append(drone1.Dst);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append("Location: ").Append(drone2.GpsListPosition).Append(" Asnd: ").Append(drone2.AssignedCoordinates).Append(" Unit OK: ").Append(drone2.Dst);
            droneInformation.AppendLine();

            // Row 11
            s = droneInformation.Length;
            droneInformation.Append("X: ").Append(drone1.LocationX).Append(" Y: ").Append(drone1.LocationY).Append(" Z: ").Append(drone1.LocationZ);
            droneInformation.Append(' ', Math.Max(clbs - (droneInformation.Length - s), 0));
            if (slu) droneInformation.Append("X: ").Append(drone2.LocationX).Append(" Y: ").Append(drone2.LocationY).Append(" Z: ").Append(drone2.LocationZ);
            droneInformation.AppendLine();
        }

        IEnumerator<bool> GenListDisplay()
        {
            if (!listHeaderGenerated)
            {
                // Chained appends bypass string.Format allocation completely
                displayTextList.Append(secondary_tag).Append(" Mining Grid Status [").Append(jobname)
                    .Append("] - GMDC ").Append(ver).Append(" ").Append(icon).AppendLine();

                // Using \n inside the string for double spacing is faster than multiple calls
                displayTextList.Append("\nRemaining bores: ").Append(boresRemaining)
                    .Append(" - Current Index: ").Append(currentGPSIndex).AppendLine();

                listHeaderGenerated = true;
            }

            for (int i = 0; i < gridBoreFinished.Count; i++)
            {
                // EXCEPTION FIX: Safe 'for' loop to prevent crashes during yield pauses
                drone_namer = "";
                for (int dIndex = 0; dIndex < droneID.Count; dIndex++)
                {
                    DroneData drone;
                    if (!Swarm.TryGetValue(droneID[dIndex], out drone)) continue;

                    if (!gridBoreOccupied[i])
                    {
                        drone_namer = "";
                    }
                    else if (i == drone.GpsListPosition)
                    {
                        drone_namer = drone.Name;
                        drone.AssignsCount++;
                    }

                    if (drone.AssignsCount > 1)
                    {
                        gridBoreOccupied[i] = false;
                    }
                    drone.AssignsCount = 0;
                }

                if (!gridBoreFinished[i])
                {
                    // Zero-allocation integer and boolean appending
                    displayTextList.Append("\nGrid Index: ").Append(i)
                        .Append(" - Occupied: ").Append(gridBoreOccupied[i]) // Fixed spelling from Occuipied
                        .Append(" - Assigned: ").Append(drone_namer);
                }

                if (i == gridBoreFinished.Count - 1)
                {
                    listGeneratorFinished = true;
                }

                percent_list = ((double)i / (double)gridBoreFinished.Count) * 100;
                yield return false;
            }
            yield return true;
        }
        IEnumerator<bool> GenGrdPosits(Vector3D centerPoint, Vector3D planeNormal, double gridSize, int numPointsX, int numPointsY, bool coreout, bool perimeterOnly = false)
        {
            // debugcount++;
            // initgridcount++;
            // List<Vector3D> grdPositins = new List<Vector3D>();

            int gridcount_inner = 0;
            int gridcount_outer = 0;
            int gridcount = 0;
            int core_numpoints_x = 0;
            int core_numpoints_y = 0;

            Vector3D xAxis = Vector3D.CalculatePerpendicularVector(planeNormal);
            Vector3D yAxis = Vector3D.Cross(planeNormal, xAxis);
            Vector3D halfOffsetX = (numPointsX - 1) * 0.5 * gridSize * xAxis;
            Vector3D halfOffsetY = (numPointsY - 1) * 0.5 * gridSize * yAxis;

            // Calculate total points for the outer layer accurately based on mode
            if (perimeterOnly)
            {
                int innerX = numPointsX - 2;
                int innerY = numPointsY - 2;
                if (innerX < 0) innerX = 0;
                if (innerY < 0) innerY = 0;

                // Perimeter count = Total Area minus the Inner Core Area
                gridcount_outer = (numPointsX * numPointsY) - (innerX * innerY);
            }
            else
            {
                gridcount_outer = numPointsX * numPointsY;
            }

            // Temporary list to hold main grid positions for sorting
            List<Vector3D> tempPositions = new List<Vector3D>();

            for (int i = 0; i < numPointsX; i++)
            {
                for (int j = 0; j < numPointsY; j++)
                {
                    // If mapping perimeter only, skip any point that isn't on a boundary edge
                    if (perimeterOnly && !(i == 0 || i == numPointsX - 1 || j == 0 || j == numPointsY - 1))
                    {
                        continue;
                    }

                    Vector3D position = centerPoint + i * gridSize * xAxis - j * gridSize * yAxis - halfOffsetX + halfOffsetY;
                    tempPositions.Add(position);
                }
                yield return false;
            }

            // Sort the main grid outward from the center point
            tempPositions.Sort((a, b) => Vector3D.DistanceSquared(a, centerPoint).CompareTo(Vector3D.DistanceSquared(b, centerPoint)));

            // Add the sorted positions to your global arrays
            foreach (var pos in tempPositions)
            {
                gridBorePosition.Add(pos);
                gridBoreOccupied.Add(false);
                gridBoreFinished.Add(false);
            }

            // Yielding after the loop to preserve execution limits
            yield return false;

            // Only map the coreout sub-grid if we are not in perimeter-only mode
            if (coreout && !perimeterOnly)
            {
                core_numpoints_x = numPointsX - 1;
                core_numpoints_y = numPointsY - 1;
                Vector3D halfOffsetX_core = (core_numpoints_x - 1) * 0.5 * gridSize * xAxis;
                Vector3D halfOffsetY_core = (core_numpoints_y - 1) * 0.5 * gridSize * yAxis;

                if (core_numpoints_x < 1) core_numpoints_x = 1;
                if (core_numpoints_y < 1) core_numpoints_y = 1;

                gridcount_inner = core_numpoints_x * core_numpoints_y;

                if (gridcount_inner >= 1)
                {
                    // Temporary list to hold coreout positions for sorting
                    List<Vector3D> tempCorePositions = new List<Vector3D>();

                    for (int i = 0; i < core_numpoints_x; i++)
                    {
                        for (int j = 0; j < core_numpoints_y; j++)
                        {
                            Vector3D position = centerPoint + i * gridSize * xAxis - j * gridSize * yAxis - halfOffsetX_core + halfOffsetY_core;
                            tempCorePositions.Add(position);
                        }
                        yield return false;
                    }

                    // Sort the coreout grid outward from the center point
                    tempCorePositions.Sort((a, b) => Vector3D.DistanceSquared(a, centerPoint).CompareTo(Vector3D.DistanceSquared(b, centerPoint)));

                    // Add the sorted coreout positions to your global arrays
                    foreach (var pos in tempCorePositions)
                    {
                        gridBorePosition.Add(pos);
                        gridBoreOccupied.Add(false);
                        gridBoreFinished.Add(false);
                    }

                    yield return false;
                }
            }

            gridcount = gridcount_inner + gridcount_outer;
            percent_grid = (double)gridBorePosition.Count / (double)gridcount;

            // sbtexttemp.AppendLine($"{gridcount} {grid_bore_positions.Count} {gridcount}");
            if (gridBorePosition.Count == gridcount)
            {
                gridInitialisationComplete = true;
            }
            else
            {
                gridInitialisationComplete = false;
            }

            yield return true;
        }
        private float GetScaleToFit(string text, float targetWidth, float baseScale, float charWidthAtBase)
        {
            // Estimate total width: char count * width per char at scale 1.0
            float textWidth = text.Length * charWidthAtBase;

            // If text is wider than the target, return a reduced scale; otherwise keep it at baseScale
            if (textWidth > targetWidth)
            {
                return (targetWidth / textWidth) * baseScale;
            }

            return baseScale;
        }
        IEnumerator<bool> BuildSprites(Vector3D centerPoint, Vector3D planeNormal, double gridSize, int numPointsX, int numPointsY, bool coreout, bool rotateToHome = true)
        {
            // Define the usable width of your viewport
            float usableWidth = _viewport.Width * 0.9f; // 90% of viewport width
            float charWidth = 14.0f; // Adjust this value until the text fits perfectly
            int sprite_total = 0;
            int drone_total = 0;

            var Viewport_scale = 0.4f;
            var Viewport_size_y = _viewport.Height * Viewport_scale;
            var Viewport_size_x = _viewport.Width * Viewport_scale;
            var view_spacer_x = Viewport_size_x / numPointsX;
            var view_spacer_y = Viewport_size_y / numPointsY;
            var sizer = new Vector2(view_spacer_x, view_spacer_y);
            var width_real = gridSize * numPointsX;
            var height_real = gridSize * numPointsY;
            var scale_factor_x = (float)(Viewport_size_x / width_real) * 1.5f;
            var scale_factor_y = (float)(Viewport_size_y / height_real) * 1.5f;

            if (gridBorePosition.Count > 0)
            {

                Vector3D xAxis = Vector3D.CalculatePerpendicularVector(planeNormal);
                Vector3D yAxis = Vector3D.Cross(planeNormal, xAxis);

                // --- Viewport Rotation Matrix Setup ---
                bool doRotation = rotateToHome && remoteControlActual != null;
                double cosRot = 1.0;
                double sinRot = 0.0;

                if (doRotation)
                {
                    scale_factor_x *= 0.8f;
                    scale_factor_y *= 0.8f;
                    if (sizer != new Vector2(view_spacer_x * 0.8f, view_spacer_y * 0.8f))
                    {
                        sizer = new Vector2(view_spacer_x * 0.8f, view_spacer_y * 0.8f);
                    }
                    Vector3D LocalRCHome = remoteControlActual.GetPosition();
                    Vector3D relativePointHome = LocalRCHome - centerPoint;

                    // Get local 2D planar coordinates
                    double xPlanarHome = Vector3D.Dot(relativePointHome, xAxis);
                    double yPlanarHome = Vector3D.Dot(relativePointHome, yAxis);

                    // Screen Y is inverted
                    double cxHome = xPlanarHome;
                    double cyHome = -yPlanarHome;

                    double magHome = Math.Sqrt(cxHome * cxHome + cyHome * cyHome);

                    if (magHome > 0.0001)
                    {
                        // Calculate angle required to point Home straight up (-PI/2 radians)
                        double currentTheta = Math.Atan2(cyHome, cxHome);
                        double targetTheta = -Math.PI / 2.0;
                        double alpha = targetTheta - currentTheta;

                        cosRot = Math.Cos(alpha);
                        sinRot = Math.Sin(alpha);
                    }
                    else
                    {
                        if (sizer != new Vector2(view_spacer_x, view_spacer_y))
                        {
                            sizer = new Vector2(view_spacer_x, view_spacer_y);
                        }
                        doRotation = false; // RC is at dead center, no rotation possible
                    }
                }

                // --- Draw Home Chevron ---
                float screenRadius = Math.Min(_viewport.Size.X, _viewport.Size.Y) / 2 * 0.8f;
                if (remoteControlActual != null)
                {
                    Vector3D LocalRCHome = remoteControlActual.GetPosition();
                    Vector3D relativePointHome = LocalRCHome - centerPoint;
                    string ImageHome = "AH_BoreSight";
                    var home_colour = Color.Purple;
                    var alpha_bytes_home = 1.0f;

                    double xPlanarHome = Vector3D.Dot(relativePointHome, xAxis);
                    double yPlanarHome = Vector3D.Dot(relativePointHome, yAxis);

                    double cxHome = xPlanarHome;
                    double cyHome = -yPlanarHome;

                    if (doRotation)
                    {
                        if (sizer != new Vector2(view_spacer_x * 0.8f, view_spacer_y * 0.8f))
                        {
                            sizer = new Vector2(view_spacer_x * 0.8f, view_spacer_y * 0.8f);
                        }
                        double xRot = cxHome * cosRot - cyHome * sinRot;
                        double yRot = cxHome * sinRot + cyHome * cosRot;
                        cxHome = xRot;
                        cyHome = yRot;
                    }

                    var CentXHome = (float)cxHome * scale_factor_x;
                    var CentYHome = (float)cyHome * scale_factor_y;
                    float mag = (float)Math.Sqrt(CentXHome * CentXHome + CentYHome * CentYHome);

                    float normX = mag > 0 ? CentXHome / mag : 0;
                    float normY = mag > 0 ? CentYHome / mag : -1;

                    var positionHome = _viewport.Center - screenRadius * new Vector2(normX, normY);
                    float rotationHome = (float)Math.Atan2(normY, normX) + (float)Math.PI;

                    var spriteHomeDirection = new MySprite()
                    {
                        Type = SpriteType.TEXTURE,
                        Data = ImageHome,
                        Position = positionHome,
                        RotationOrScale = rotationHome,
                        Size = sizer,
                        Color = home_colour.Alpha(alpha_bytes_home),
                        Alignment = TextAlignment.CENTER
                    };

                    if (mag != 0.0f)
                    {
                        sprites.Add(spriteHomeDirection);
                        spriteCounter++;
                    }
                }


                textSpriteBuffer.Clear();
                textSpriteBuffer.Append("--- ").Append(secondary_tag).Append(" Mining Grid Status ---");
                var text_position = new Vector2(256, 20) + _viewport.Position;
                datatemp = textSpriteBuffer.ToString();
                scale1 = GetScaleToFit(datatemp, usableWidth, 1.0f, charWidth);

                var spriteText = new MySprite()
                {
                    Type = SpriteType.TEXT,
                    Data = datatemp,
                    Position = text_position,
                    RotationOrScale = scale1,
                    Size = sizer,
                    Color = Color.WhiteSmoke.Alpha(1.0f),
                    Alignment = TextAlignment.CENTER,
                    FontId = "White"
                };
                sprites.Add(spriteText);
                text_position = new Vector2(256, 60) + _viewport.Position;
                textSpriteBuffer.Clear();
                textSpriteBuffer.Append("[").Append(jobname).Append("] - Total Bores: ").Append(totalMiningRuns).Append(" - Remaining: ").Append(boresRemaining).Append(" - Drones: ").Append(totalDronesMining).Append(" ").Append("(").Append(totalDronesActive).Append(")");
                datatemp = textSpriteBuffer.ToString();
                scale1 = GetScaleToFit(datatemp, usableWidth, 1.0f, charWidth);
                spriteText = new MySprite()
                {
                    Type = SpriteType.TEXT,
                    Data = datatemp,
                    Position = text_position,
                    RotationOrScale = scale1,
                    Size = sizer,
                    Color = Color.WhiteSmoke.Alpha(1.0f),
                    Alignment = TextAlignment.CENTER,
                    FontId = "White"
                };
                sprites.Add(spriteText);
                textSpriteBuffer.Clear();
                datatemp = "";

                // --- Draw Grid Bores ---
                for (int i = 0; i < gridBorePosition.Count; i++)
                {
                    sprite_total++;
                    Vector3D relativePoint = gridBorePosition[i] - centerPoint;
                    double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                    double yPlanar = Vector3D.Dot(relativePoint, yAxis);

                    double cx = xPlanar;
                    double cy = -yPlanar;

                    // Apply mathematical rotation to the coordinates
                    if (doRotation)
                    {
                        double xRot = cx * cosRot - cy * sinRot;
                        double yRot = cx * sinRot + cy * cosRot;
                        cx = xRot;
                        cy = yRot;
                    }

                    var CentX = -(float)cx;
                    var CentY = -(float)cy;

                    string Image = gridBoreFinished[i] ? "CircleHollow" : "Circle";
                    var alpha_bytes = gridBoreOccupied[i] ? 1.0f : 0.5f;
                    var bore_colour = gridBoreOccupied[i] ? Color.LightSkyBlue : Color.DeepSkyBlue;
                    var position = new Vector2(CentX * scale_factor_x, CentY * scale_factor_y) + _viewport.Center;

                    var sprite = new MySprite()
                    {
                        Type = SpriteType.TEXTURE,
                        Data = Image,
                        Position = position,
                        Size = sizer,
                        Color = bore_colour.Alpha(alpha_bytes),
                        Alignment = TextAlignment.CENTER
                    };
                    sprites.Add(sprite);
                    percent_list_vis = (((double)i + (double)1) / ((double)gridBorePosition.Count)) * 100;
                    spriteCounter++;
                    yield return false;
                }



                // --- Draw Drones ---
                if (droneID.Count > 0)
                {
                    drone_total = 0;

                    // EXCEPTION FIX: Using a standard 'for' loop over the ID list instead of 'foreach' on the dictionary.
                    for (int dIndex = 0; dIndex < droneID.Count; dIndex++)
                    {
                        DroneData drone;
                        // If the drone somehow doesn't exist in the dictionary, skip to the next one safely.
                        if (!Swarm.TryGetValue(droneID[dIndex], out drone)) continue;

                        double drone_locale_x = 0.0;
                        double drone_locale_y = 0.0;
                        double drone_locale_z = 0.0;
                        drone_total++;

                        if (!double.TryParse(drone.LocationX, out drone_locale_x)) { drone_locale_x = 0.0; }
                        if (!double.TryParse(drone.LocationY, out drone_locale_y)) { drone_locale_y = 0.0; }
                        if (!double.TryParse(drone.LocationZ, out drone_locale_z)) { drone_locale_z = 0.0; }

                        Vector3D Drone_point = new Vector3D(drone_locale_x, drone_locale_y, drone_locale_z);
                        Vector3D relativePoint = Drone_point - centerPoint;
                        double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                        double yPlanar = Vector3D.Dot(relativePoint, yAxis);

                        double cx = xPlanar;
                        double cy = -yPlanar;

                        if (doRotation)
                        {
                            double xRot = cx * cosRot - cy * sinRot;
                            double yRot = cx * sinRot + cy * cosRot;
                            cx = xRot;
                            cy = yRot;
                        }

                        var CentX = -(float)cx;
                        var CentY = -(float)cy;
                        string Image_drone = "Circle";
                        var bore_colour_drone = Color.Gray;
                        var alpha_val = 1.0f;

                        if (drone.ControlStatus.Contains("Docked") || drone.ControlStatus.Contains("Undocked") || drone.ControlStatus.Contains("Docking") || drone.ControlStatus.Contains("Undocking"))
                        {
                            alpha_val = 0.25f;
                        }

                        if (drone.IsMining)
                        {
                            if (drone.ControlStatus.Contains("Min")) { bore_colour_drone = Color.Purple; }
                            else if (drone.ControlStatus.Contains("Exit")) { bore_colour_drone = Color.Orange; }
                            else if (drone.ControlStatus.Contains("RTB: Ready")) { bore_colour_drone = Color.Green; }
                            else if (drone.ControlStatus.Contains("Undock")) { bore_colour_drone = Color.Yellow; }
                            else { bore_colour_drone = Color.Navy; }
                            alpha_val = 1.0f;
                        }
                        if (drone.DamageState == "DMG")
                        {
                            bore_colour_drone = Color.Red;
                        }

                        var position = new Vector2(CentX * scale_factor_x, CentY * scale_factor_y) + _viewport.Center;

                        var sprite = new MySprite()
                        {
                            Type = SpriteType.TEXTURE,
                            Data = Image_drone,
                            Position = position,
                            Size = sizer * 0.8f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER
                        };
                        sprites.Add(sprite);

                        if (drone.CargoFull.Contains("True") || drone.RechargeRequest.Contains("True"))
                        {
                            if (drone.RechargeRequest.Contains("True") && drone.CargoFull.Contains("True")) { bore_colour_drone = Color.White; }
                            else if (drone.RechargeRequest.Contains("True")) { bore_colour_drone = Color.YellowGreen; }
                            else if (drone.CargoFull.Contains("True")) { bore_colour_drone = Color.RosyBrown; }

                            var sprite_layer_h = new MySprite()
                            {
                                Type = SpriteType.TEXTURE,
                                Data = "CircleHollow",
                                Position = position,
                                Size = sizer * 0.8f,
                                Color = bore_colour_drone.Alpha(alpha_val),
                                Alignment = TextAlignment.CENTER
                            };
                            sprites.Add(sprite_layer_h);
                            spriteCounter++;
                        }

                        if (drone.ControlStatus.Contains("Recharg") || drone.ControlStatus.Contains("Unload") || drone.RechargeRequest.Contains("True"))
                        {
                            bore_colour_drone = Color.Yellow;
                            var sprite_layer = new MySprite()
                            {
                                Type = SpriteType.TEXTURE,
                                Data = "IconEnergy",
                                Position = position,
                                Size = sizer * 0.8f,
                                Color = bore_colour_drone.Alpha(alpha_val),
                                Alignment = TextAlignment.CENTER
                            };
                            sprites.Add(sprite_layer);
                            spriteCounter++;
                        }

                        bore_colour_drone = Color.WhiteSmoke;
                        var sprite_name = new MySprite()
                        {
                            Type = SpriteType.TEXT,
                            Data = $"{drone.Name}- ({drone.ChargeStorage}%)",
                            Position = position,
                            RotationOrScale = 0.3f, // Text scale remains untouched, orientation stays perfectly level
                            Size = sizer * 0.5f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER,
                            FontId = "White"
                        };
                        sprites.Add(sprite_name);

                        percent_list_drones = ((double)drone_total / (double)droneID.Count) * 100;
                        spriteCounter++;
                        yield return false; // Pauses execution safely, dictionary modification won't crash the for-loop
                    }
                }


                if (droneID.Count == 0)
                {
                    frame_generator_finished = (sprite_total == gridBorePosition.Count);
                }
                else if (droneID.Count > 0)
                {
                    if (sprite_total == gridBorePosition.Count && drone_total == droneID.Count)
                    {
                        frame_generator_finished = true;
                        drone_total = 0;
                    }
                    else
                    {
                        frame_generator_finished = false;
                    }
                }

                yield return true;
            }
        }

        public void DrawSprites(ref MySpriteDrawFrame frame)
        {
            if (spriteCounter >= spritecount_limit_insert && !spriteInsert)
            {
                var banger = new MySprite();
                frame.Add(banger);
                sbtexttemp.AppendLine("Frame shift");
                spriteInsert = true;
                spriteCounter++;
            }
            // Create background sprite
            var sprite = new MySprite()
            {
                Type = SpriteType.TEXTURE,
                Data = "Grid",
                Position = _viewport.Center,
                Size = _viewport.Size,
                Color = sV.ScriptForegroundColor.Alpha(0.0f),
                Alignment = TextAlignment.CENTER
            };
            frame.Add(sprite);
            spriteCounter++;

            for (int i = 0; i < sprites.Count; i++)
            {
                frame.Add(sprites[i]);
            }
        }

        int CountTrueValuesList(List<bool> list)
        {
            int truCnt = 0;

            foreach (bool value in list)
            {
                if (value)
                {
                    truCnt++;
                }
            }
            return truCnt;
        }
        int CountTrueValues(string propertyName)
        {
            int truCnt = 0;
            foreach (DroneData drone in Swarm.Values)
            {
                if (propertyName == "Dst" && drone.Dst) truCnt++;
                if (propertyName == "IsMining" && drone.IsMining) truCnt++;
            }
            return truCnt;
        }
        int CountIntegerValues(string propertyName, int val)
        {
            int truCnt = 0;

            foreach (DroneData drone in Swarm.Values)
            {
                if (propertyName == "GpsListPosition" && drone.GpsListPosition == val) truCnt++;
                if (propertyName == "ControlSequence" && drone.ControlSequence == val) truCnt++;
            }

            return truCnt;
        }
        int CountStatusValues(string propertyName, string textval)
        {
            int trueCount = 0;

            foreach (DroneData drone in Swarm.Values)
            {
                if (propertyName == "ControlStatus" && drone.ControlStatus == textval) trueCount++;
            }
            return trueCount;
        }
        public void droneCommandBuilder(string cdata_1, string xpos, string ypos, string zpos, string cdata_5, string cmdo, string data_6, string idepth, string xpos2, string ypos2, string zpos2)
        {
            const string baseFormat = "GPS:{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:";
            const string astFormat = "GPS:PAD:{0}:{1}:{2}:#FF75C9F1:";

            c.Clear().EnsureCapacity((prospectAlignTargetValid || customDataAlignTargetValid) ? 120 : 80); // ~80 chars base, ~40 more if asteroid
            c.AppendFormat(baseFormat, cdata_1, xpos, ypos, zpos, cdata_5, cmdo, data_6, idepth);
            if (prospectAlignTargetValid || customDataAlignTargetValid) c.AppendFormat(astFormat, xpos2, ypos2, zpos2);
        }
        void transmitToDrone(DroneData drone)
        {
            IGC.SendBroadcastMessage(tx_chan, drone.TransmissionOutput, TransmissionDistance.TransmissionDistanceMax);
        }
        void GetStoredData(string input)
        {
            if (!string.IsNullOrEmpty(input) && !string.IsNullOrWhiteSpace(input)) // Check null or empty
            {
                var str = "";
                string gridstats = "";
                if (_ini.TryParse(input))
                {
                    str = _ini.Get("configuration", "runargument").ToString().Trim();
                    runargument = str;
                    str = _ini.Get("configuration", "ship grid tag").ToString();
                    secondary = str;
                    str = _ini.Get("jobdata", "gridstatus").ToString().Trim();
                    gridstats = str;
                }
                Echo("Loading grid data");
                if (setupComplete)
                {
                    string[] str_data = gridstats.Split(';');
                    for (int i = 0; i < str_data.Length; i++)
                    {
                        if (string.IsNullOrEmpty(str_data[i])) continue; // Skip empty entries (e.g., trailing semicolon)

                        string[] str_datai = str_data[i].Split(':');
                        if (str_datai.Length >= 2) // Minimum for bn:bc
                        {
                            int bn, bc;
                            bool bnParsed = int.TryParse(str_datai[0], out bn);
                            bool bcParsed = int.TryParse(str_datai[1], out bc);

                            gridBoreFinished.Add(bnParsed && bn > 0); // Default false if unparsed
                            gridBoreOccupied.Add(bcParsed && bc > 0); // Default false if unparsed

                            if (str_datai.Length >= 5) // Full bn:bc:x:y:z
                            {
                                double x = double.TryParse(str_datai[2], out bx) ? bx : 0.0;
                                double y = double.TryParse(str_datai[3], out by) ? by : 0.0;
                                double z = double.TryParse(str_datai[4], out bz) ? bz : 0.0;
                                gridBorePosition.Add(new Vector3D(x, y, z));
                            }
                            else
                            {
                                gridBorePosition.Add(new Vector3D(0, 0, 0)); // Default position if incomplete
                            }
                        }
                    }
                }
            }
        }
        void GetSavedJobData(IMyProgrammableBlock block)
        {

            var str = "";
            string gridstats = "";
            Echo("Loading grid data");
            if (setupComplete)
            {
                _ini.Clear();
                if (_ini.TryParse(block.CustomData.ToString()))
                {
                    str = _ini.Get("jobdata", "gridstatus").ToString().Trim();
                    gridstats = str;
                    string[] str_data = gridstats.Split(';');
                    for (int i = 0; i < str_data.Length; i++)
                    {
                        if (string.IsNullOrEmpty(str_data[i])) continue; // Skip empty entries (e.g., trailing semicolon)

                        string[] str_datai = str_data[i].Split(':');
                        if (str_datai.Length >= 2) // Minimum for bn:bc
                        {
                            int bn, bc;
                            bool bnParsed = int.TryParse(str_datai[0], out bn);
                            bool bcParsed = int.TryParse(str_datai[1], out bc);

                            gridBoreFinished.Add(bnParsed && bn > 0); // Default false if unparsed
                            gridBoreOccupied.Add(bcParsed && bc > 0); // Default false if unparsed

                            if (str_datai.Length >= 5) // Full bn:bc:x:y:z
                            {
                                double x = double.TryParse(str_datai[2], out bx) ? bx : 0.0;
                                double y = double.TryParse(str_datai[3], out by) ? by : 0.0;
                                double z = double.TryParse(str_datai[4], out bz) ? bz : 0.0;
                                gridBorePosition.Add(new Vector3D(x, y, z));
                            }
                            else
                            {
                                gridBorePosition.Add(new Vector3D(0, 0, 0)); // Default position if incomplete
                            }
                        }
                    }
                }
                _ini.Set(gmdccategory, "loadsave", false);
            }
            block.CustomData = _ini.ToString();
            _ini.Clear();
        }
        void runicon(int state)
        {
            if (state == 0)
            {
                icon = ".---";
            }
            if (state == 1)
            {
                icon = "-.--";
            }
            if (state == 2)
            {
                icon = "--.-";
            }
            if (state == 3)
            {
                icon = "---.";
            }
        }
        void StateShifter()
        {
            stateshift++;
            if (stateshift > 3)
            {
                stateshift = 0;
            }
            runicon(stateshift);
        }
        public void ParseAndApplyArguments(string input)
        {
            // --- Step 1: Handle Empty Input (Using the simpler IsNullOrWhiteSpace check) ---
            if (string.IsNullOrWhiteSpace(input))
            {
                sbtexttemp.AppendLine("No arguments provided, using defaults.");
                drone_tag = "SWRM_D";
                drone_length = 2.6;
                drone_clear_offset = 12.0; //drill clear mode distance offset
                secondary = ""; //vessel/rig name (optional)
                return;
            }

            string[] dronecontrolleronfigdata = input.Split(',');

            // Check if the split array is unexpectedly empty (though covered by the initial check)
            if (dronecontrolleronfigdata.Length == 0)
            {
                sbtexttemp.AppendLine("No arguments provided, using defaults.");
                // Use a consistent set of defaults or return immediately.
                return;
            }
            if (dronecontrolleronfigdata.Length >= 1 && !string.IsNullOrWhiteSpace(dronecontrolleronfigdata[0]))
            {
                drone_tag = dronecontrolleronfigdata[0].ToString().Trim();
            }
            else
            {
                drone_tag = "SWRM_D"; // Default C if argument is missing or empty
            }
            if (dronecontrolleronfigdata.Length > 1)
            {
                if (string.IsNullOrWhiteSpace(dronecontrolleronfigdata[0]))
                {
                    sbtexttemp.AppendLine("Drone tag is empty, using default: SWRM_D");
                }
                for (int i = 1; i < dronecontrolleronfigdata.Length; i++)
                {
                    if (dronecontrolleronfigdata[i] != null)
                    {
                        if (!string.IsNullOrWhiteSpace(dronecontrolleronfigdata[i]))
                        {
                            string[] argCommand = dronecontrolleronfigdata[i].Split('=');
                            //string[] argCommand_End = input.Split(',');
                            if (argCommand.Length > 0)
                            {
                                if (argCommand[0].Contains(commandArg1))
                                {
                                    if (argCommand.Length > 1)
                                    {
                                        if (argCommand[1] != null && !string.IsNullOrWhiteSpace(argCommand[1]))
                                        {
                                            secondary = argCommand[1].Trim();
                                        }
                                        else
                                        {
                                            secondary = ""; // Default if argument is missing or empty
                                        }
                                    }
                                }
                                if (argCommand[0].Contains(commandArg2))
                                {
                                    if (argCommand.Length > 1)
                                    {
                                        if (argCommand[1] != null && !string.IsNullOrWhiteSpace(argCommand[1]))
                                        {

                                            if (!double.TryParse(argCommand[1].Trim(), out drone_length))
                                            {
                                                drone_length = 2.6; // Set to default on fail
                                            }

                                        }
                                        else
                                        {
                                            drone_length = 2.6; // Default if argument is missing or empty
                                        }
                                    }
                                }
                                if (argCommand[0].Contains(commandArg3))
                                {
                                    if (argCommand.Length > 1)
                                    {
                                        if (argCommand[1] != null && !string.IsNullOrWhiteSpace(argCommand[1]))
                                        {
                                            if (!double.TryParse(argCommand[1].Trim(), out drone_clear_offset))
                                            {
                                                drone_clear_offset = 12.0; // Set to default on fail
                                            }

                                        }
                                        else
                                        {
                                            drone_clear_offset = 12.0; // Default if argument is missing or empty
                                        }
                                    }
                                }
                                if (argCommand[0].Contains(commandArg4))
                                {
                                    if (argCommand.Length > 1)
                                    {
                                        if (argCommand[1] != null && !string.IsNullOrWhiteSpace(argCommand[1]))
                                        {
                                            if (!bool.TryParse(argCommand[1].Trim(), out rotateHome))
                                            {
                                                rotateHome = true; // Set to default on fail
                                            }

                                        }
                                        else
                                        {
                                            rotateHome = true; // Default if argument is missing or empty
                                        }
                                    }
                                }
                                if (argCommand[0].Contains(commandArg5))
                                {
                                    if (argCommand.Length > 1)
                                    {
                                        if (argCommand[1] != null && !string.IsNullOrWhiteSpace(argCommand[1]))
                                        {
                                            if (!int.TryParse(argCommand[1].Trim(), out drones_per_screen))
                                            {
                                                drones_per_screen = 8; // Set to default on fail
                                            }

                                        }
                                        else
                                        {
                                            drones_per_screen = 8; // Default if argument is missing or empty
                                        }
                                    }
                                }

                            }
                        }
                    }

                }
            }
            RefreshDroneGates();
        }
        public void SetupSystem()
        {
            ClearAllNonEmptyLists();
            #region setup_system
            Echo("Running Setup");
            IMyGridTerminalSystem gts = GridTerminalSystem as IMyGridTerminalSystem;
            Echo("Loading Names");
            antennaTagName = "[" + drone_tag + " " + comms + "]";
            lightsTagName = "[" + drone_tag + " " + comms + "]";
            dp_mn_tag = "[" + drone_tag + " " + MainS + " " + dspy + "]";
            dp_drn_tag = "[" + drone_tag + " " + DroneS + " " + dspy + "]";
            dp_lst_tag = "[" + drone_tag + " " + LstS + " " + dspy + "]";
            interfaceTag = "[" + drone_tag + " " + IntfS + "]";
            dp_vis_tag = "[" + drone_tag + " " + GrphS + " " + dspy + "]";
            secondary_tag = "[" + secondary + "]";
            rxChannelDrone = drone_tag + " " + replyC;
            rxChannelProspector = drone_tag + " " + prospC;
            tx_recall_channel = drone_tag + " " + commandRecall;
            txDronePingChannel = "[" + drone_tag + "]" + " " + pingMessage;
            txDroneSyncChannel = "[" + drone_tag + "]" + " " + syncC;
            syncMessage = secondary;
            Echo("Clearning Lists");
            rotors_all.Clear();
            rotorAdvancedStators_all.Clear();
            pistons_all.Clear();

            bool blockfinder = false;
            gts.GetBlocksOfType<IMyMotorStator>(rotors_all, b => b.TopGrid == Me.CubeGrid);

            if (rotors_all.Count <= 0)
            {
                Echo("Rotor top grid not found, checking advanced rotors");
            }

            if (rotors_all.Count > 0)
            {
                if (rotors_all[0] != null)
                {
                    meCubeGrid = rotors_all[0].CubeGrid;
                    Echo("Local cubegrid found - rotor");
                    blockfinder = true;
                }
            }

            gts.GetBlocksOfType<IMyMotorAdvancedStator>(rotorAdvancedStators_all, b => b.TopGrid == Me.CubeGrid);

            if (rotorAdvancedStators_all.Count <= 0)
            {
                Echo("Rotor top grid not found, checking advanced rotors");
            }
            if (rotorAdvancedStators_all.Count > 0)
            {
                if (rotorAdvancedStators_all[0] != null)
                {
                    meCubeGrid = rotorAdvancedStators_all[0].CubeGrid;
                    Echo("Local cubegrid found - advanced rotor/hinge");
                    blockfinder = true;
                }
            }


            gts.GetBlocksOfType<IMyPistonBase>(pistons_all, b => b.TopGrid == Me.CubeGrid);

            if (pistons_all.Count <= 0)
            {
                Echo("Rotor top grid not found, checking pistons");
            }
            if (pistons_all.Count > 0)
            {
                if (pistons_all[0] != null)
                {

                    meCubeGrid = pistons_all[0].CubeGrid;
                    Echo("Local cubegrid found - piston");
                    blockfinder = true;
                }
            }


            if (rotorAdvancedStators_all.Count == 0 && rotors_all.Count == 0 && pistons_all.Count == 0)
            {
                meCubeGrid = Me.CubeGrid;
                Echo("Local cubegrid found - PB");
                blockfinder = false;
            }

            rotors_all.Clear();
            rotorAdvancedStators_all.Clear();
            pistons_all.Clear();

            Echo("Stage 1");

            gridBorePosition.Clear();

            droneMining.Clear();

            Echo("Stage 2");

            sprites.Clear();
            remoteControlAll.Clear();
            remoteControlTag.Clear();

            droneID.Clear();
            cl.Clear();
            cl2.Clear();
            Echo("Stage 3");
            miningCoordinatesNew.Clear();
            displayTextMain.Clear();
            displayTextList.Clear();
            droneInformation.Clear();
            c.Clear();
            jxt.Clear();
            customDataString.Clear();
            Echo("ListClear Complete");
            bores_regen = false;
            listenDrones = IGC.RegisterBroadcastListener(rxChannelDrone);
            listenProspector = IGC.RegisterBroadcastListener(rxChannelProspector);
            for (int i = 0; i < 12; i++)
            {
                cl.Add("");
                cl2.Add("");
            }
            antennaAll.Clear();
            antennaTag.Clear();
            if (blockfinder)
            {
                gts.GetBlocksOfType<IMyRadioAntenna>(antennaAll, b => b.CubeGrid == meCubeGrid);
                if (antennaAll.Count > 0)
                {
                    for (int i = 0; i < antennaAll.Count; i++)
                    {

                        if (antennaAll[i].CustomName.Contains(antennaTagName) || antennaAll[i].CustomName.Contains(comms))
                        {
                            string checker = antennaAll[i].CustomData;
                            //drone_custom_data_check(checker, i);
                            if (string.IsNullOrEmpty(drone_tag) || string.IsNullOrWhiteSpace(drone_tag))
                            {
                                Echo($"Invalid name for drone_tag {drone_tag} please add drone tag to GMDC antenna custom data '<yourdronetaghere>:<Yourshiptaghere>:' e.g. 'SWRM_D:Atlas:'");
                                return;

                            }
                            antennaAll[i].CustomName = $"GMDC Antenna {secondary_tag} {antennaTagName}";
                            antennaTag.Add(antennaAll[i]);
                        }
                    }
                }
                antennaAll.Clear();
            }
            gts.GetBlocksOfType<IMyRadioAntenna>(antennaAll, b => b.CubeGrid == Me.CubeGrid);
            if (antennaAll.Count > 0)
            {
                for (int i = 0; i < antennaAll.Count; i++)
                {

                    if (antennaAll[i].CustomName.Contains(antennaTagName) || antennaAll[i].CustomName.Contains(comms))
                    {
                        string checker = antennaAll[i].CustomData;
                        //drone_custom_data_check(checker, i);
                        if (string.IsNullOrEmpty(drone_tag) || string.IsNullOrWhiteSpace(drone_tag))
                        {
                            Echo($"Invalid name for drone_tag {drone_tag} please add drone tag to GMDC antenna custom data '<yourdronetaghere>:<Yourshiptaghere>:' e.g. 'SWRM_D:Atlas:'");
                            return;

                        }
                        antennaAll[i].CustomName = $"GMDC Antenna {secondary_tag} {antennaTagName}";
                        antennaTag.Add(antennaAll[i]);
                    }
                }
            }
            antennaAll.Clear();

            Me.CustomName = $"GMDC Programmable Block {secondary_tag} {antennaTagName}";

            remoteControlAll.Clear();
            remoteControlTag.Clear();
            if (blockfinder)
            {
                gts.GetBlocksOfType<IMyRemoteControl>(remoteControlAll, b => b.CubeGrid == Me.CubeGrid);
                if (remoteControlAll.Count > 0)
                {
                    for (int i = 0; i < remoteControlAll.Count; i++)
                    {
                        if (remoteControlAll[i].CustomName.Contains(antennaTagName) || remoteControlAll[i].CustomName.Contains(comms))
                        {
                            remoteControlAll[i].CustomName = $"GMDC Remote Control {secondary_tag} {antennaTagName}";
                            remoteControlTag.Add(remoteControlAll[i]);
                        }
                    }
                }
                remoteControlAll.Clear();
            }
            gts.GetBlocksOfType<IMyRemoteControl>(remoteControlAll, b => b.CubeGrid == meCubeGrid);
            if (remoteControlAll.Count > 0)
            {
                for (int i = 0; i < remoteControlAll.Count; i++)
                {
                    if (remoteControlAll[i].CustomName.Contains(antennaTagName) || remoteControlAll[i].CustomName.Contains(comms))
                    {
                        remoteControlAll[i].CustomName = $"GMDC Remote Control {secondary_tag} {antennaTagName}";
                        remoteControlTag.Add(remoteControlAll[i]);
                    }
                }
            }
            remoteControlAll.Clear();

            lightsAll.Clear();
            lightsTag.Clear();
            if (blockfinder)
            {
                gts.GetBlocksOfType<IMyLightingBlock>(lightsAll, b => b.CubeGrid == Me.CubeGrid);
                if (lightsAll.Count > 0)
                {
                    for (int i = 0; i < lightsAll.Count; i++)
                    {
                        if (lightsAll[i].CustomName.Contains(lightsTagName) || lightsAll[i].CustomName.Contains(comms))
                        {
                            lightsAll[i].CustomName = $"GMDC Indicator Light {secondary_tag} {lightsTagName}";
                            lightsTag.Add(lightsAll[i]);
                        }
                    }
                }
                lightsAll.Clear();
            }
            gts.GetBlocksOfType<IMyLightingBlock>(lightsAll, b => b.CubeGrid == meCubeGrid);
            for (int i = 0; i < lightsAll.Count; i++)
            {
                if (lightsAll[i].CustomName.Contains(lightsTagName) || lightsAll[i].CustomName.Contains(comms))
                {
                    lightsAll[i].CustomName = $"GMDC Indicator Light {secondary_tag} {lightsTagName}";
                    lightsTag.Add(lightsAll[i]);
                }
            }
            lightsAll.Clear();

            display_all.Clear();
            display_tag_main.Clear();
            display_tag_list.Clear();
            display_tag_drone.Clear();
            display_tag_vis.Clear();
            myTextSurfaces_d1.Clear();
            myTextSurfaces_d2.Clear();
            myTextSurfaces_d3.Clear();
            myTextSurfaces_d4.Clear();
            if (blockfinder)
            {
                gts.GetBlocksOfType<IMyTerminalBlock>(display_all, b => b.CubeGrid == Me.CubeGrid);
                if (display_all.Count > 0)
                {
                    for (int i = 0; i < display_all.Count; i++)
                    {
                        if (display_all[i].CustomName.Contains(dp_mn_tag))
                        {
                            display_tag_main.Add(display_all[i]);
                            myTextSurfaces_d1.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfM));
                        }
                        if (display_all[i].CustomName.Contains(dp_drn_tag))
                        {
                            display_tag_drone.Add(display_all[i]);
                            myTextSurfaces_d2.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfD));
                        }
                        if (display_all[i].CustomName.Contains(dp_lst_tag))
                        {
                            display_tag_list.Add(display_all[i]);
                            myTextSurfaces_d3.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfL));
                        }
                        if (display_all[i].CustomName.Contains(dp_vis_tag))
                        {
                            display_tag_vis.Add(display_all[i]);
                            myTextSurfaces_d4.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfV));
                        }
                    }
                }
                display_all.Clear();
            }
            gts.GetBlocksOfType<IMyTerminalBlock>(display_all, b => b.CubeGrid == meCubeGrid);
            if (display_all.Count > 0)
            {
                for (int i = 0; i < display_all.Count; i++)
                {
                    if (display_all[i].CustomName.Contains(dp_mn_tag))
                    {
                        display_tag_main.Add(display_all[i]);
                        myTextSurfaces_d1.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfM));
                    }
                    if (display_all[i].CustomName.Contains(dp_drn_tag))
                    {
                        display_tag_drone.Add(display_all[i]);
                        myTextSurfaces_d2.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfD));
                    }
                    if (display_all[i].CustomName.Contains(dp_lst_tag))
                    {
                        display_tag_list.Add(display_all[i]);
                        myTextSurfaces_d3.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfL));
                    }
                    if (display_all[i].CustomName.Contains(dp_vis_tag))
                    {
                        display_tag_vis.Add(display_all[i]);
                        myTextSurfaces_d4.Add(((IMyTextSurfaceProvider)display_all[i]).GetSurface(srfV));
                    }
                }
            }
            display_all.Clear();

            programblockAll.Clear();
            interfacePBTag.Clear();
            if (blockfinder)
            {
                gts.GetBlocksOfType<IMyProgrammableBlock>(programblockAll, b => b.CubeGrid == Me.CubeGrid);
                if (programblockAll.Count > 0)
                {
                    for (int i = 0; i < programblockAll.Count; i++)
                    {
                        if (programblockAll[i].CustomName.Contains(interfaceTag) || programblockAll[i].CustomName.Contains(IntfS))
                        {
                            programblockAll[i].CustomName = $"GMDI Programmable Block {secondary_tag} {interfaceTag}";
                            interfacePBTag.Add(programblockAll[i]);
                        }
                    }
                }
                programblockAll.Clear();
            }
            gts.GetBlocksOfType<IMyProgrammableBlock>(programblockAll, b => b.CubeGrid == meCubeGrid);
            if (programblockAll.Count > 0)
            {
                for (int i = 0; i < programblockAll.Count; i++)
                {
                    if (programblockAll[i].CustomName.Contains(interfaceTag) || programblockAll[i].CustomName.Contains(IntfS))
                    {
                        programblockAll[i].CustomName = $"GMDI Programmable Block {secondary_tag} {interfaceTag}";
                        interfacePBTag.Add(programblockAll[i]);
                    }
                }
            }
            programblockAll.Clear();

            droneMessagesBuffer.Clear();
            prospectorMessagesBuffer.Clear();

            if (Runtime.UpdateFrequency == UpdateFrequency.Update1)
            {
                game_factor = 1;
            }
            if (Runtime.UpdateFrequency == UpdateFrequency.Update10)
            {
                game_factor = 10;
            }
            if (Runtime.UpdateFrequency == UpdateFrequency.Update100)
            {
                game_factor = 100;
            }

            if (myTextSurfaces_d4.Count > 0)
            {
                if (myTextSurfaces_d4[0] != null)
                {
                    sV = myTextSurfaces_d4[0];
                    if (sV.ContentType != ContentType.SCRIPT)
                    {
                        sbtexttemp.AppendLine("Correcting visualiser display");
                        sV.ContentType = ContentType.SCRIPT;
                        sV.Script = "";
                        Visport_OK = true;
                        _viewport = new RectangleF((sV.TextureSize - sV.SurfaceSize) / 2f, sV.SurfaceSize);
                    }
                }
            }
            if (sV == null)
            {
                Echo($"Panel:'{srfV}' on '{dp_vis_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (display_tag_vis.Count <= 0 || display_tag_vis[0] == null)
            {
                Echo($"Display with tag '{dp_vis_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
                Visport_OK = false;
            }
            if (display_tag_vis.Count > 0 && display_tag_vis[0] != null)
            {
                Echo($"Display with tag '{dp_vis_tag.Replace("[", "[[").Replace("]", "]]")}' found");
                _viewport = new RectangleF((sV.TextureSize - sV.SurfaceSize) / 2f, sV.SurfaceSize);
                Visport_OK = true;
            }

            if (myTextSurfaces_d1.Count > 0)
            {
                for (int i = 0; i < myTextSurfaces_d1.Count; i++)
                {
                    if (myTextSurfaces_d1[i] != null)
                    {
                        if (myTextSurfaces_d1[i].ContentType != ContentType.TEXT_AND_IMAGE)
                        {
                            myTextSurfaces_d1[i].ContentType = ContentType.TEXT_AND_IMAGE;
                            myTextSurfaces_d1[i].Alignment = TextAlignment.LEFT;
                            myTextSurfaces_d1[i].FontSize = 0.50f;
                            myTextSurfaces_d1[i].Font = "White";
                        }
                    }
                }
            }
            if (myTextSurfaces_d2.Count > 0)
            {
                for (int i = 0; i < myTextSurfaces_d2.Count; i++)
                {
                    if (myTextSurfaces_d2[i] != null)
                    {
                        if (myTextSurfaces_d2[i].ContentType != ContentType.TEXT_AND_IMAGE)
                        {
                            myTextSurfaces_d2[i].ContentType = ContentType.TEXT_AND_IMAGE;
                            myTextSurfaces_d2[i].Alignment = TextAlignment.LEFT;
                            myTextSurfaces_d2[i].FontSize = 0.296f;
                            myTextSurfaces_d2[i].Font = "Monospace";
                        }
                    }
                }
            }
            if (myTextSurfaces_d3.Count > 0)
            {
                for (int i = 0; i < myTextSurfaces_d3.Count; i++)
                {
                    if (myTextSurfaces_d3[i] != null)
                    {
                        if (myTextSurfaces_d3[i].ContentType != ContentType.TEXT_AND_IMAGE)
                        {
                            myTextSurfaces_d3[i].ContentType = ContentType.TEXT_AND_IMAGE;
                            myTextSurfaces_d3[i].Alignment = TextAlignment.LEFT;
                            myTextSurfaces_d3[i].FontSize = 0.50f;
                            myTextSurfaces_d3[i].Font = "White";
                        }
                    }
                }
            }
            if (myTextSurfaces_d4.Count > 0)
            {
                for (int i = 0; i < myTextSurfaces_d4.Count; i++)
                {
                    if (myTextSurfaces_d4[i] != null)
                    {
                        if (myTextSurfaces_d4[i].ContentType != ContentType.SCRIPT)
                        {
                            myTextSurfaces_d4[i].ContentType = ContentType.SCRIPT;
                            myTextSurfaces_d4[i].Script = "";
                        }
                    }
                }
            }

            d1_tag = dp_mn_tag.Replace("[", "[[").Replace("]", "]]");
            d2_tag = dp_drn_tag.Replace("[", "[[").Replace("]", "]]");
            d3_tag = dp_lst_tag.Replace("[", "[[").Replace("]", "]]");
            d4_tag = dp_vis_tag.Replace("[", "[[").Replace("]", "]]");
            channel_tag_display = drone_tag.Replace("[", "[[").Replace("]", "]]");
            secondary_tag_display = secondary_tag.Replace("[", "[[").Replace("]", "]]");
            interface_display = interfaceTag.Replace("[", "[[").Replace("]", "]]");

            #endregion
        }
        public void ComponentPresenceCheck()
        {
            #region presence_check
            if (antennaTag.Count <= 0 || antennaTag[0] == null)
            {
                Echo($"Antenna with tag: '{antennaTagName.Replace("[", "[[").Replace("]", "]]")}' not found.");
                setupComplete = !setupComplete;
                return;
            }
            antennaActual = antennaTag[0];
            if (remoteControlTag.Count <= 0 || remoteControlTag[0] == null)
            {
                Echo($"remote control with tag: '{antennaTagName.Replace("[", "[[").Replace("]", "]]")}' not found.");
                setupComplete = !setupComplete;
                return;
            }
            remoteControlActual = remoteControlTag[0];


            if (lightsTag.Count <= 0 || lightsTag[0] == null)
            {
                Echo($"Indicator light with tag: '{lightsTagName.Replace("[", "[[").Replace("]", "]]")}' not found.");
                setupComplete = !setupComplete;
                return;
            }
            lightIndicatorActual = lightsTag[0];
            lightIndicatorActual.SetValue("Color", Cred);

            if (interfacePBTag.Count <= 0 || interfacePBTag[0] == null)
            {
                Echo($"Interface PB with tag: '{interface_display}' not found.");
                setupComplete = !setupComplete;
                return;
            }

            if (display_tag_main.Count <= 0 || display_tag_main[0] == null)
            {
                Echo($"Display with tag '{dp_mn_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (display_tag_main.Count > 0 && display_tag_main[0] != null)
            {
                sM = ((IMyTextSurfaceProvider)display_tag_main[0]).GetSurface(srfM);
            }
            if (sM == null)
            {
                Echo($"Panel:'{srfM}' on '{dp_mn_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (display_tag_list.Count <= 0 || display_tag_list[0] == null)
            {
                Echo($"Display with tag '{dp_mn_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (display_tag_list.Count > 0 && display_tag_list[0] != null)
            {
                sL = ((IMyTextSurfaceProvider)display_tag_list[0]).GetSurface(srfL);
            }
            if (sL == null)
            {
                Echo($"Panel:'{srfL}' on '{dp_lst_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (display_tag_drone.Count <= 0 || display_tag_drone[0] == null)
            {
                Echo($"Display with tag '{dp_lst_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }


            if (display_tag_drone.Count > 0 && display_tag_drone[0] != null)
            {
                sD = ((IMyTextSurfaceProvider)display_tag_drone[0]).GetSurface(srfD);
            }
            if (sM == null)
            {
                Echo($"Panel:'{srfD}' on '{dp_drn_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (interfacePBTag.Count <= 0 || interfacePBTag[0] == null)
            {
                Echo($"Interface PB with tag: '{interfaceTag.Replace("[", "[[").Replace("]", "]]")}' not found.");
            }

            if (display_tag_vis.Count > 0 && display_tag_vis[0] != null)
            {

                sV = ((IMyTextSurfaceProvider)display_tag_vis[0]).GetSurface(srfV);
                if (sV.ContentType != ContentType.SCRIPT)
                {
                    Echo("Correcting vis");
                    sV.ContentType = ContentType.SCRIPT;
                    sV.Script = "";
                    Visport_OK = true;
                    _viewport = new RectangleF((sV.TextureSize - sV.SurfaceSize) / 2f, sV.SurfaceSize);
                }
            }
            if (sV == null)
            {
                Echo($"Panel:'{srfV}' on '{dp_vis_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
            }
            if (display_tag_vis.Count <= 0 || display_tag_vis[0] == null)
            {
                Echo($"Display with tag '{dp_vis_tag.Replace("[", "[[").Replace("]", "]]")}' not found");
                Visport_OK = false;
            }
            #endregion
        }

        public void ProcessReceivedDroneMessageToDroneLists()
        {
            DroneData drone;
            // Ensure we are working with a clean string
            string safeDroneName = receivedDroneName.Trim();

            // Check if the dictionary does NOT contain this drone yet
            if (!Swarm.ContainsKey(safeDroneName))
            {

                // 1. Create the new drone object
                DroneData newDrone = new DroneData();

                // 2. Populate its data
                newDrone.Name = safeDroneName;
                newDrone.DamageState = receivedDroneDamageStatus;
                newDrone.TunnelFinished = receivedDroneTunnelFinished;
                newDrone.ControlStatus = receivedDroneStatus;
                newDrone.Docked = receivedDroneDocked;
                newDrone.Undocked = receivedDroneUndocked;
                newDrone.Autopiloting = recived_drone_autopilot;
                newDrone.GpsListPosition = -1;
                newDrone.GpsCoordinates = remoteControlActual.GetPosition();
                newDrone.BoreDepth = rc_dn_drl_dpth;
                newDrone.BoreDepthCurrent = rc_dn_drl_crnt;
                newDrone.MineDepthStartStatus = rc_dn_drl_strt;
                newDrone.LocationX = rc_locx;
                newDrone.LocationY = rc_locy;
                newDrone.LocationZ = rc_locz;
                newDrone.ChargeStorage = rc_dn_chg;
                newDrone.GasStorage = rc_dn_gas;
                newDrone.OreStorage = rc_dn_str;
                newDrone.IsMining = false;
                newDrone.AssignedCoordinates = false;
                newDrone.ControlSequence = 0;
                newDrone.RecallSequence = 0;
                newDrone.TransmissionOutput = "";
                newDrone.IsReady = false;
                newDrone.MustWait = true;
                newDrone.Dcs = 0.0;
                newDrone.Dst = true;
                newDrone.TransmissionStatus = true;
                newDrone.RecallList = false;
                newDrone.ResetFunction = false;
                newDrone.AssignsCount = 0;
                newDrone.CargoFull = rc_dn_cargo_full;
                newDrone.RechargeRequest = rc_dn_rchg_req;
                newDrone.AutoPilotEnabled = rc_auto_pilot_enabled;
                newDrone.Autodock = recievedDroneAutdock;
                newDrone.DockingReady = recievedDroneDockingReady;
                newDrone.AssignedGates = new List<IMyDoor>();
                ScanDoors(safeDroneName, newDrone.AssignedGates);

                // 3. Add the fully built drone to the dictionary
                droneID.Add(newDrone.Name);
                Swarm.Add(safeDroneName, newDrone);

            }
            else if (Swarm.TryGetValue(safeDroneName, out drone))
            {
                //drone.Name = safeDroneName;
                drone.DamageState = receivedDroneDamageStatus;
                drone.TunnelFinished = receivedDroneTunnelFinished;
                drone.ControlStatus = receivedDroneStatus;
                drone.Docked = receivedDroneDocked;
                drone.Undocked = receivedDroneUndocked;
                drone.Autopiloting = recived_drone_autopilot;
                drone.GpsListPosition = recieved_drone_list_position;
                drone.BoreDepth = rc_dn_drl_dpth;
                drone.BoreDepthCurrent = rc_dn_drl_crnt;
                drone.MineDepthStartStatus = rc_dn_drl_strt;
                drone.LocationX = rc_locx;
                drone.LocationY = rc_locy;
                drone.LocationZ = rc_locz;
                drone.ChargeStorage = rc_dn_chg;
                drone.GasStorage = rc_dn_gas;
                drone.OreStorage = rc_dn_str;
                drone.Dcs = rc_d_cn;
                drone.TransmissionStatus = true;
                drone.CargoFull = rc_dn_cargo_full;
                drone.RechargeRequest = rc_dn_rchg_req;
                drone.AutoPilotEnabled = rc_auto_pilot_enabled;
                drone.Autodock = recievedDroneAutdock;
                drone.DockingReady = recievedDroneDockingReady;
                if (drone.Dcs <= bclm)
                {
                    drone.Dst = false;
                }
                if (drone.Dcs > bclm)
                {
                    drone.Dst = true;
                }
                droneMessageConfirmed = true;
                incomingName = safeDroneName;
            }
            #region drone_message_data_processing
            /*
                            drone.Name = receivedDroneName;
                            drone.DamageState = receivedDroneDamageStatus;
                            drone.TunnelFinished = receivedDroneTunnelFinished;
                            drone.ControlStatus = receivedDroneStatus;
                            drone.Docked = receivedDroneDocked;
                            drone.Undocked = receivedDroneUndocked;
                            droneAutopiloting[i] = recived_drone_autopilot;
                            droneBoreDepth[i] = (rc_dn_drl_dpth);
                            droneBoreDepthCurrent[i] = rc_dn_drl_crnt;
                            drone_mine_depth_start_status[i] = rc_dn_drl_strt;
                            drone.GpsListPosition = recieved_drone_list_position;
                            drone.LocationX = rc_locx;
                            drone.LocationY = rc_locy;
                            drone.LocationZ = rc_locz;
                            drone.ChargeStorage = rc_dn_chg;
                            drone_gas_storage[i] = rc_dn_gas;
                            drone_ore_storage[i] = rc_dn_str;
                            drone.Dcs = rc_d_cn;
                            drone.TransmissionStatus = true;
                            drone.CargoFull = rc_dn_cargo_full;
                            drone.RechargeRequest = rc_dn_rchg_req;
                            drone_auto_pilot_enabled[i] = rc_auto_pilot_enabled;
                            droneAutodock[i] = recievedDroneAutdock;
                            droneDockingReady[i] = recievedDroneDockingReady;
                            if (drone.Dcs <= bclm)
                            {
                                dst[i] = false;
                            }
                            if (drone.Dcs > bclm)
                            {
                                dst[i] = true;
                            }
                            droneMessageConfirmed = true;
                            receivedDroneNameIndex = i;
                            break;
 
            }
            */
            #endregion
        }

        void update_display()  // Extracted from drone_processing
        {
            displayTextMain.Clear().EnsureCapacity(512); // ~400-600 chars typical

            displayTextMain.Append("GMDC ").Append(ver).Append(" ").Append(secondary_tag)
                .Append(" [").Append(drone_tag).Append("] [").Append(jobname).Append("] Running ")
                .Append(icon).Append(" ").AppendLine();

            // Using \n inside AppendLine is highly efficient for double spacing
            displayTextMain.AppendLine("------------------------------\n");

            displayTextMain.Append("Total drones detected: ").Append(Swarm.Count).AppendLine();

            // Replaced the string-interpolated ternary operator with a zero-allocation if block
            displayTextMain.Append("Drones active: ").Append(totalDronesActive).Append(" - Fault: ").Append(totalDronesDamaged);
            if (dronesLaunchedStatus)
            {
                displayTextMain.Append(" (Max: ").Append(maxActiveDronesCount).Append(" (").Append(dronesInFlightFactor).Append(")) Hard limit: ").Append(dronesActiveHardLimit);
            }
            displayTextMain.AppendLine();

            displayTextMain.Append("Docking: ").Append(t_drn_dckg).Append(" Docked: ").Append(t_drn_dck)
                .Append(" - Unload: ").Append(t_drn_unload).Append(" Recharge: ").Append(t_drn_rechg)
                .Append(" Idle: ").Append(t_drn_idle_docked).AppendLine("  ");

            displayTextMain.Append("Undocking: ").Append(t_drn_udckg).Append(" Undocked: ").Append(t_drn_udck)
                .Append(" - Idle: ").Append(t_drn_idle_undocked).Append(" Nav: ").Append(t_drn_nav)
                .Append(" Mining: ").Append(t_drn_mine).Append(" Exit: ").Append(t_drn_exit).AppendLine();

            displayTextMain.AppendLine();

            displayTextMain.Append("Surface distance: ").Append(safe_dstvl).AppendLine("m");

            displayTextMain.Append("Drill depth: ").Append(drillLength).Append("m (").Append(drillLength + safe_dstvl).AppendLine("m)");

            displayTextMain.Append("Req. ignore depth: ").Append(ignoreDepth).Append("m (Drone length: ").Append(drone_length).AppendLine("m)");

            displayTextMain.Append("Ignore depth: ").Append(safe_dstvl + drone_length - drone_clear_offset + ignoreDepth)
                .Append("m (Drill Start: ").Append((drillLength + safe_dstvl) - (ignoreDepth + safe_dstvl + drone_length - drone_clear_offset)).AppendLine("m)\n");

            displayTextMain.Append("Command: ").Append(commandAsk).Append(" Reset: ").Append(generalReset).AppendLine();

            displayTextMain.Append("Status: ").Append(screenStatus).AppendLine("\n");

            displayTextMain.Append("Target Coordinates [").Append(jobname).AppendLine("]:");

            // Appending Vector3D implicitly calls its ToString(), which is fine here
            displayTextMain.AppendLine(miningGPSCoordinates.ToString());

            if (prospectAlignTargetValid || customDataAlignTargetValid)
            {
                displayTextMain.Append("Align Coordinates:\n").AppendLine(alignGPSCoordinates.ToString());
            }
            if (myTextSurfaces_d1.Count > 0)
            {
                for (int i = 0; i < myTextSurfaces_d1.Count; i++)
                {
                    if (myTextSurfaces_d1[i] != null)
                    {
                        myTextSurfaces_d1[i].WriteText(displayTextMain);
                    }
                }
            }
        }

        public void ClearAllNonEmptyLists()
        {
            // Clear Space Engineers block lists (reference types)
            ClearReferenceLists(remoteControlAll, remoteControlTag);
            ClearReferenceLists(antennaAll, antennaTag);
            ClearReferenceLists(lightsAll, lightsTag);
            ClearReferenceLists(display_all, display_tag_main, display_tag_list,
                               display_tag_drone, display_tag_vis);
            ClearReferenceLists(programblockAll, interfacePBTag);

            // Clear other reference type lists (strings)
            ClearReferenceLists(




                               cl, cl2
                                 );

            // Clear value type lists (bool)
            ClearValueLists(droneMining,

                            gridBoreOccupied, gridBoreFinished);


            // Clear struct lists (Vector3D)
            ClearValueLists(gridBorePosition);

            // Clear struct lists (MySprite)
            ClearValueLists(sprites);

            // Clear struct lists (MyIGCMessage)
            ClearValueLists(droneMessagesBuffer, prospectorMessagesBuffer);

            // Clear StringBuilder instances with null checks
            if (miningCoordinatesNew?.Length > 0) miningCoordinatesNew.Clear();
            if (displayTextMain?.Length > 0) displayTextMain.Clear();
            if (displayTextList?.Length > 0) displayTextList.Clear();
            if (droneInformation?.Length > 0) droneInformation.Clear();
            if (c?.Length > 0) c.Clear();
            if (jxt?.Length > 0) jxt.Clear();
            if (customDataString?.Length > 0) customDataString.Clear();
        }

        // For reference types (classes)
        private void ClearReferenceLists<T>(params List<T>[] lists) where T : class
        {
            foreach (var list in lists)
            {
                if (list?.Count > 0) // Safe null check
                {
                    list.Clear();
                }
            }
        }

        // For value types and structs (no constraint)
        private void ClearValueLists<T>(params List<T>[] lists)
        {
            foreach (var list in lists)
            {
                if (list?.Count > 0) // Safe null check
                {
                    list.Clear();
                }
            }
        }
        //program end
        public void StoreRawInput(string inputString, IMyTerminalBlock block, string INI_SECTION = "GMDCJobData", string INI_KEY = "Jobinfo")
        {
            var iniBuilder = new MyIni();
            iniBuilder.Clear();
            if (iniBuilder.TryParse(block.CustomData.ToString()))
            {
                iniBuilder.Set(INI_SECTION, INI_KEY, inputString);
            }
            else
            {
                iniBuilder.Set(INI_SECTION, INI_KEY, inputString); ;
            }
            block.CustomData = iniBuilder.ToString();
            iniBuilder.Clear();
            sbtexttemp.AppendLine($"Raw input stored successfully in [{INI_SECTION}] {INI_KEY}.");
        }

        public void ScanDoors(string droneid, List<IMyDoor> gateslist)
        {
            gateslist.Clear();
            GridTerminalSystem.GetBlocksOfType<IMyDoor>(gateslist, b => b.CubeGrid == meCubeGrid && b.CustomName.Contains(droneid));
        }


        public void RefreshDroneGates()
        {
            _allDoorsCache.Clear();
            GridTerminalSystem.GetBlocksOfType<IMyDoor>(_allDoorsCache, b => b.CubeGrid == meCubeGrid);

            foreach (var drone in Swarm.Values)
            {
                drone.AssignedGates.Clear();

                for (int i = 0; i < _allDoorsCache.Count; i++)
                {
                    var door = _allDoorsCache[i];
                    if (
                        door.CustomName.IndexOf(drone.Name, StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        drone.AssignedGates.Add(door);
                    }
                }
            }
            _allDoorsCache.Clear();
        }


        public class DroneData
        {
            // Identifiers & Status
            public string Name;
            public string DamageState;
            public string TunnelFinished;
            public string ControlStatus;

            // Docking & Autopilot
            public string Docked;
            public string Undocked;
            public string Autopiloting;
            public string Autodock;
            public string DockingReady;
            public string AutoPilotEnabled;

            // Navigation & Coordinates
            public int GpsListPosition;
            public Vector3D GpsCoordinates;
            public string LocationX;
            public string LocationY;
            public string LocationZ;

            // Mining & Depth
            public string BoreDepth;
            public string BoreDepthCurrent;
            public string MineDepthStartStatus;
            public bool IsMining;

            // Storage & Resources
            public string ChargeStorage;
            public string GasStorage;
            public string OreStorage;
            public string CargoFull;
            public string RechargeRequest;

            // State Machine Flags
            public bool AssignedCoordinates;
            public int ControlSequence;
            public int RecallSequence;
            public string TransmissionOutput;
            public bool IsReady;
            public bool MustWait;
            public double Dcs;
            public bool Dst;
            public bool TransmissionStatus;
            public bool RecallList;
            public bool ResetFunction;
            public int AssignsCount;
            public bool canlaunch;
            public List<IMyDoor> AssignedGates = new List<IMyDoor>();
        }

    }

}

