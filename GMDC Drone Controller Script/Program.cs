using Sandbox.Game.Components;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.Dynamic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Input;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        // R e a d m e
        // -----------
        // GMDC Drone controller 504B refactor
        // 


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
        double drone_clear_offset = 6.6; //drill clear mode distance offset
        string secondary = ""; //vessel/rig name (optional)

        //display surface indexes
        int srfM = 0;
        int srfL = 0;
        int srfD = 0;
        int srfV = 0;
        int drones_per_screen = 8;
        int droneUndockDelayTime = 60;
        int undock_delay_limit = 120;

        //Drone Comms
        int droneCommunicationsProcessingDelay = 1;
        int droneCommunicationsPingDelay = 18;

        #endregion

        #region static_variables
        //visualiser settings
        int spriteCountLimit = 500;
        int spritecount_limit_insert = 250;
        //statics
        int game_factor = 10;
        string ver = "V0.509B";
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
        bool found = false;
        bool generalReset;
        bool miningGridValid = false;
        List<bool> droneMustWait;
        double ignoreDepth = 0.0;
        double safe_dstvl = 0.0;
        bool prospectTargetValid = false;
        bool prospectAlignTargetValid = false;
        bool customDataAlignTargetValid = false;
        string commandAsk;
        //string customData1;
        //string customData2;
        //string customData3;
        //string customData4;
        string customData5;
        //string customData6;
        string customData7;
        string customData8;
        string customData9;
        //string customData10;
        //string customData11;
        //string customData12;
        //string customData13;
        //string customData14;
        //string customData15;
        //string customData16;
        //string customData17;
        //string customData18;
        //string customData19;
        //string customData20;
        //string customData21;
        //string customData22;
        //string remoteControlCustomData1 = "";
        //string remoteControlCustomData2 = "";
        //string remoteControlCustomData3 = "";
        //string remoteControlCustomData4 = "";
        //string remoteControlCustomData5 = "";
        //string remoteControlCustomData6 = "";
        //string remoteControlCustomData7 = "";
        //string remoteControlCustomData8 = "";
        //string remoteControlCustomData9 = "";
        //string remoteControlCustomData10 = "";
        //string remoteControlCustomData11 = "";
       // string remoteControlCustomData12 = "";

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
        private IEnumerator<bool> gridCoroutine, listCoroutine, visCoroutine;
        IMyRadioAntenna antennaActual;
        IMyLightingBlock lightIndicatorActual;
        IMyRemoteControl remoteControlActual;
        IMyProgrammableBlock pbInterfaceActual;
        Vector3D miningGPSCoordinates;
        Vector3D gridCentreGPSCoordinates;
        //Vector3D next_gps_crds;
        Vector3D targetGPSCoordinates;
        Vector3D alignGPSCoordinates;
        Vector3D planeNrml;
        StringBuilder miningCoordinatesNew;
        StringBuilder c;
        List<Vector3D> drone_location;
        List<string> droneName;
        List<string> droneDamageState;
        List<string> droneTunnelFinished;
        List<string> droneControlStatus;
        List<string> droneDocked;
        List<string> droneUndocked;
        List<string> droneAutopiloting;
        List<string> droneBoreDepth;
        List<string> droneBoreDepthCurrent;
        List<string> drone_mine_depth_start_status;
        List<int> droneGPSListPosition;
        List<bool> droneReady;
        List<string> drone_location_x;
        List<string> drone_location_y;
        List<string> drone_location_z;
        List<string> drone_charge_storage;
        List<string> drone_gas_storage;
        List<string> drone_ore_storage;
        List<string> drone_cargo_full;
        List<string> drone_recharge_request;
        List<string> drone_auto_pilot_enabled;
        List<string> droneAutodock;
        List<string> droneDockingReady;
        List<int> drone_assigns_count;
        List<double> dcs;
        List<bool> droneAssignedCoordinates;
        List<bool> droneRecallList;
        List<Vector3D> droneGPSCoordinates;
        List<int> droneControlSequence;
        List<int> droneRecallSequence;
        List<bool> droneResetFunction;
        List<string> droneTranmissionOutput;
        List<Vector3D> gridBorePosition;
        List<bool> gridBoreOccupied;
        List<bool> gridBoreFinished;
        List<string> cl;
        List<string> cl2;
        List<int> tla;
        List<int> rst;
        List<string> fct;
        List<bool> dst;
        List<bool> droneTransmissionStatus;
        //int cbval = 0;
        //bool clbt = false;
        int gridBoresCompleted;
        int gpsGridPositionValue = -1;
        string drone_namer = "";
        StringBuilder droneInformation;
        StringBuilder displayTextMain;
        StringBuilder displayTextList;
        StringBuilder jxt;
        List<bool> droneMining;
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
        List<IMyRemoteControl> remoteControlAll, remoteControlTag;
        List<IMyRadioAntenna> antennaAll, antennaTag;
        List<IMyLightingBlock> lightsAll, lightsTag;
        List<IMyTerminalBlock> display_all, display_tag_main, display_tag_list, display_tag_drone, display_tag_vis;
        List<IMyProgrammableBlock> programblockAll, interfacePBTag;
        IMyTextSurface sD, sM, sL, sV;
        RectangleF _viewport;
        StringBuilder sb;
        int totalDronesDamaged = 0;
        int totalDronesUnknown = 0;
        //int t_dn_ok = 0;
        string g1;
        string g2;
        int di = 0;
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

        IMyBroadcastListener listenDrones, listenProspector;
        List<MyIGCMessage> droneMessagesBuffer, prospectorMessagesBuffer;
        bool prospectorMessageReceived = false;
        bool droneMessageConfirmed = false;
        //int receivedDroneNameIndex = -1;
        bool droneMessageReceived = false;
        bool Visport_OK = false;
        List<MySprite> sprites;
        int spriteCounter = 0;
        bool spriteInsert = false;
        StringBuilder customDataString, _statusBuffer;
        string _oldCustomData = "";
        Dictionary<String, Drone> droneRegistry;

        public class Drone
        {
            public string droneName;
            public string droneDamageState;
            public string droneTunnelFinished;
            public string droneControlStatus;
            public string droneDocked;
            public string droneUndocked;
            public string droneAutopiloting;
            public int droneGPSListPosition;
            public Vector3D droneGPSCoordinates;
            public string droneBoreDepth;
            public string droneBoreDepthCurrent;
            public string drone_mine_depth_start_status;
            public string drone_location_x;
            public string drone_location_y;
            public string drone_location_z;
            public string drone_charge_storage;
            public string drone_gas_storage;
            public string drone_ore_storage;
            public bool droneMining;
            public bool droneAssignedCoordinates;
            public int droneControlSequence;
            public int droneRecallSequence;
            public string droneTranmissionOutput;
            public bool droneReady;
            public bool droneMustWait;
            public double dcs;
            public bool dst;
            public bool droneTransmissionStatus;
            public bool droneRecallList;
            public bool droneResetFunction;
            public int drone_assigns_count;
            public string drone_cargo_full;
            public string drone_recharge_request;
            public string drone_auto_pilot_enabled;
            public string droneAutodock;
            public string droneDockingReady;
        }

        //private double totalRuntimeMs = 0.0;
        //private int runCount = 0;
        //private double averageRuntimeMs = 0.0;

        MyIni _ini = new MyIni();
        MyIni _antennaStore = new MyIni();
        string runargument = "";
        bool firstload = false;
        MyIni _customDataStore = new MyIni();
        //MyIni _commsData = new MyIni();
        string jobdata = "";
        string rcjobdata = "";
        string jobinfo = "Jobinfo";
        string gmdccategory = "GMDCJobData";
        //string gmdpcategory = "GMDPJobData";
        string rcjobinfo = "Jobinfo";
        //string prospectmain = "maindata";
       // string prospecttarget = "aligndata";
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
                    sb.Clear();
                    _ini.Clear();
                }
            }


        }

        public void AntennaSaveData(IMyRadioAntenna block)
        {
            _antennaStore.Clear();
            _antennaStore.Set("Configuration", "drone group tag", drone_tag);
            _antennaStore.Set("Configuration", "ship grid tag", secondary_tag);
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
            //Echo("Running Modular Main - v1");
            int startInstructions = Runtime.CurrentInstructionCount;

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
                return;
            }
            ProcessInputs(argument);
            ManageCommunications();
            UpdateMiningGrid();
            HandleDroneOperations();
            RenderDisplays();
            UpdateStatus();
            Echo($"Main Total: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateRuntimeMetrics(UpdateType updateSource)
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            double _Runtime = Runtime.LastRunTimeMs;
            //totalRuntimeMs += _Runtime;
            //runCount++;
           // if (runCount == 10)
         //   {
                //averageRuntimeMs = totalRuntimeMs / runCount;
           //     runCount = 0;
                //totalRuntimeMs = 0;
          //  }
            //Echo($"UpdateRuntimeMetrics: {Runtime.CurrentInstructionCount - startInstructions}");
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
                return;
            }
            AntennaSaveData(antennaActual);
            Echo($"GMDC {ver} Running {icon}");
            Echo($"Channel: {drone_tag.Replace("[", "[[").Replace("]", "]]")}");
            //Echo($"InitializeSystem: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ProcessInputs(string argument)
        {
            string blank = "";
            int startInstructions = Runtime.CurrentInstructionCount;
            ProcessInterface();
            HandleCommands(blank);
            //Echo($"ProcessInputs: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ManageCommunications()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            listenDrones = IGC.RegisterBroadcastListener(rxChannelDrone);
            listenProspector = IGC.RegisterBroadcastListener(rxChannelProspector);
            ProcessMessages();
            //Echo($"ManageCommunications: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateMiningGrid()
        {
            int startInstructions = Runtime.CurrentInstructionCount;

            InitializeMiningGrid();
            ValidateCustomData();
            PingDrones();
            GetRemoteControlData(remoteControlActual.CustomData, remoteControlActual);
            //Echo($"Pre-Prospect: valid= RC: {prospectTargetValid}  , ALN: {prospectAlignTargetValid}, coords= PB: {miningGPSCoordinates} RC: {targetGPSCoordinates}");
            if (prospectAlignTargetValid)
            {

            }
            if (prospectorMessageReceived)
            {


                Storage = null;
                prospectAlignTargetValid = false;
                customDataAlignTargetValid = false;
                GetRemoteControlData(remoteControlActual.CustomData, remoteControlActual);
                //Echo($"Post-Prospect: valid={prospectTargetValid}, {prospectAlignTargetValid}, coords={targetGPSCoordinates}");
                if (prospectAlignTargetValid)
                {
                    Echo($"Post-Prospect: Main PB: {miningCoordsValid} Align coords=:{alignGPSCoordinates}");
                }
                if (prospectTargetValid)
                {
                    Echo($"Formatting CustomData with: {targetGPSCoordinates.X}, {targetGPSCoordinates.Y}, {targetGPSCoordinates.Z}");
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
            GetCustomDataJobCommand(Me.CustomData, Me);
            ProcessJobGrid();
            UpdateActiveDroneLimits();
            //Echo($"UpdateMiningGrid: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void HandleDroneOperations()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (droneRegistry.Count > 0 && gridCreated && timeDelayed)
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
            //Echo($"HandleDroneOperations: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void RenderDisplays()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            DroneRenderCall();
            ListRenderCall();
            SpriteRenderCall();
            //Echo($"RenderDisplays: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateStatus()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            TimeCounterReset();
            LocalStatusUpdate(Runtime.LastRunTimeMs);
            //Echo($"UpdateStatus: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void LocalStatusUpdate(double _Runtime)
        {
            Echo($"Load: {Math.Round((_Runtime / game_tick_length) * (double)100.0, 3)}% ({Math.Round(_Runtime, 3)}ms) S#:{spriteCounter} {spriteInsert} {droneRegistry.Count}");
            Echo($"Drones #: {droneRegistry.Count}");
            Echo($"Drone comms buffer: {droneMessagesBuffer.Count} OK: {droneMessageReceived}");
            Echo($"Cycles since last broadcast: {timeCounter} ({Math.Round((((double)droneCommunicationsProcessingDelay * game_tick_length) / (double)1000) * (double)game_factor, 1)}s) {timeDelayed}");
            Echo($"Cycles since last ping: {dronePingTimerCount} ({Math.Round((((double)droneCommunicationsPingDelay * game_tick_length) / (double)1000) * (double)game_factor, 1)}s)");
            Echo($"Undock cycle timer: {undockTimer} ({Math.Round((((double)undockTimer * game_tick_length) / (double)1000) * (double)game_factor, 1)}s) ({Math.Round((((double)undock_delay_limit * game_tick_length) / (double)1000) * (double)game_factor, 1)}s)");
            Echo($"Drones Undocking: {dronesUndocking} {total_drones_undocking}");
            Echo($"Prospect comms buffer: {prospectorMessagesBuffer.Count}");
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
            if (dronePingTimerCount >= droneCommunicationsPingDelay)
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
                    visCoroutine = BuildSprites(miningGPSCoordinates, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid);
                }
                if (visCoroutine != null && !frame_generator_finished)
                {
                    // Check the current yield value
                    bool currentYield = visCoroutine.Current;

                    // If the coroutine is finished, you can perform completion logic
                    if (!visCoroutine.MoveNext())
                    {
                        // The coroutine has finished executing
                        Echo("Job rendering complete.");
                        visCoroutine = null; // Reset the coroutine
                        BuildSprites(miningGPSCoordinates, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid).Dispose();
                    }
                    else
                    {
                        // Handle intermediate status if needed
                        if (!currentYield)
                        {
                            Echo($"Rendering mining job ... {Math.Round(percent_list_vis, 1)}%  {Math.Round(percent_list_drones, 1)}%");
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
                        //Echo($"Frame reset - spritecount {spriteCounter}");
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
                    Echo("Mining list complete.");
                    listCoroutine = null; // Reset the coroutine
                    GenListDisplay().Dispose();
                }
                else
                {
                    // Handle intermediate status if needed
                    if (!currentYield)
                    {
                        Echo($"Updating mining job list... {Math.Round(percent_list, 1)}%");
                        listCoroutine.MoveNext();
                    }
                }

            }
            if (display_tag_list.Count > 0 && display_tag_list[0] != null && listGeneratorFinished)
            {
                sL.WriteText(displayTextList.ToString());
                listGeneratorFinished = false;
                displayTextList.Clear();
                listHeaderGenerated = false;
            }
        }

        private void LightStatusManagement()
        {
            if (lightIndicatorActual == null || lightsTag[0] == null)
            {
                Echo($"Indicator light missing {lightsTagName.Replace("[", "[[").Replace("]", "]]")} - early exit");
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
            if (droneGPSListPosition.Count > 0 && canReset)
            {
                droneResetStatusCount = CountIntegerValues(droneRegistry, -1);
                droneDockedStatusCount = CountStatusValues(droneControlStatus, "Docked");
            }
            if (droneRegistry.Count > 0)
            {
                if (droneResetStatusCount == droneRegistry.Count && droneDockedStatusCount == droneRegistry.Count && canReset)
                {
                    readyFlag = true;
                }
            }
        }

        private void ProcessDroneState()
        {
            #region drone_state_machine_management
            if (!string.IsNullOrWhiteSpace(receivedDroneName) && droneMessageConfirmed && droneRegistry.Count > 0)
            {
                
                Drone drone;
                Echo("grabbing drone info");
                if(droneRegistry.TryGetValue(receivedDroneName, out drone))
                {
                    Echo("infograbbed");
                }
                else
                {
                    Echo("infofailed");
                }


                //int i = receivedDroneNameIndex;

                if (canInit || canReset || drone.droneResetFunction || canLoading)
                {
                    generalReset = true;
                }
                else generalReset = false;
                //di = i;
                faultCounter = totalDronesDamaged;
                if (faultCounter < droneRegistry.Count)
                {
                    faultLightOutput = true;
                }
                else faultLightOutput = false;

                //recall sequence reset - global
                if (!drone.droneRecallList || canReset || canInit || canLoading)
                {
                    drone.droneRecallSequence = 0;
                }
                displayTextMain.Clear();

                if (drone.droneGPSListPosition > -1 && !drone.droneAssignedCoordinates)
                {
                    drone.droneGPSListPosition = -1;
                }
                if (droneRegistry.Count > 0)
                {
                    if (drone.droneGPSListPosition > -1 && drone.droneAssignedCoordinates && drone.droneControlStatus.Contains("Docked") && drone.droneDocked == "True" && drone.droneMining && drone.droneReady)
                    {
                        if (gridBoreFinished[drone.droneGPSListPosition])
                        {
                            drone.droneGPSListPosition = -1;
                            drone.droneMining = false;
                            drone.droneAssignedCoordinates = false;
                        }

                    }
                }
                //if undocked request local recall sequence flag to ON
                if (drone.droneGPSListPosition == -1 && !drone.droneAssignedCoordinates && drone.droneUndocked == "True" && drone.droneDocked == "False" && !drone.droneRecallList && !mustUndockCommand || drone.droneGPSListPosition == -1 && !drone.droneAssignedCoordinates && drone.droneUndocked == "False" && drone.droneDocked == "False" && !drone.droneRecallList && !mustUndockCommand)
                {
                    drone.droneRecallList = true;
                }
                if (drone.droneRecallList)
                {
                    droneTXRecallChannel = drone.droneName + " " + commandRecall;
                    IGC.SendBroadcastMessage(droneTXRecallChannel, commandRecall, TransmissionDistance.TransmissionDistanceMax);
                }
                if (!drone.droneRecallList)
                {
                    droneTXRecallChannel = drone.droneName + " " + commandRecall;
                    IGC.SendBroadcastMessage(droneTXRecallChannel, commandOperate, TransmissionDistance.TransmissionDistanceMax);
                }


                if (drone.droneControlStatus.Contains("Docked") && drone.droneGPSListPosition == -1 && drone.droneMining && (drone.droneControlSequence == 0 || drone.droneControlSequence == 8))
                {
                    drone.droneMining = false;
                }
                if ((totalDronesMining) >= boresRemaining && !drone.droneMining && gridBoresCompleted <= totalMiningRuns || boresRemaining == 0 && !drone.droneMining)
                {
                    if (!dronesLaunchedStatus || dronesUndocking)
                    {
                        drone.droneMustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive > maxActiveDronesCount || dronesUndocking)
                    {
                        drone.droneMustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive <= maxActiveDronesCount)
                    {
                        drone.droneMustWait = false;
                    }
                }
                else if ((totalDronesMining) < boresRemaining && gridBoresCompleted < totalMiningRuns || drone.droneMining && (totalDronesMining) <= boresRemaining)
                {
                    if (!dronesLaunchedStatus)
                    {
                        drone.droneMustWait = false;
                    }
                    if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount)
                    {
                        drone.droneMustWait = false;
                    }

                    if (dronesLaunchedStatus && totalDronesActive > maxActiveDronesCount || dronesUndocking)
                    {
                        drone.droneMustWait = true;
                    }
                }
                if (drone.droneGPSListPosition == -1 && (totalDronesMining) >= boresRemaining || dronesUndocking)
                {
                    if (!dronesLaunchedStatus)
                    {
                        drone.droneMustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive >= maxActiveDronesCount || dronesUndocking)
                    {
                        drone.droneMustWait = true;
                    }
                    if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount || dronesUndocking)
                    {
                        drone.droneMustWait = true;
                    }
                }
                if (drone.droneGPSListPosition > -1 && drone.droneGPSListPosition < gridBorePosition.Count)
                {
                    if (gridBoreOccupied[drone.droneGPSListPosition] && !drone.droneMining)
                    {
                        if (!dronesLaunchedStatus || dronesUndocking)
                        {
                            drone.droneMustWait = true;
                        }
                        if (dronesLaunchedStatus && totalDronesActive >= maxActiveDronesCount || dronesUndocking)
                        {
                            drone.droneMustWait = true;
                        }
                        if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount || dronesUndocking)
                        {
                            drone.droneMustWait = true;
                        }
                    }
                    else if (gridBoresCompleted < totalMiningRuns && !gridBoreOccupied[drone.droneGPSListPosition] && !gridBoreFinished[drone.droneGPSListPosition] && !drone.droneMining)
                    {
                        if (!dronesLaunchedStatus)
                        {
                            drone.droneMustWait = false;
                        }
                        if (dronesLaunchedStatus && totalDronesActive < maxActiveDronesCount)
                        {
                            drone.droneMustWait = false;
                        }
                        if (dronesLaunchedStatus && totalDronesActive >= maxActiveDronesCount || dronesUndocking)
                        {
                            drone.droneMustWait = true;
                        }
                    }
                    if (!gridBoreFinished[drone.droneGPSListPosition])
                    {
                        int queued_count = CountIntegerValues(droneRegistry, drone.droneGPSListPosition);
                        if (gridBoreOccupied[drone.droneGPSListPosition] && queued_count == 0)
                        {
                            gridBoreOccupied[drone.droneGPSListPosition] = false;
                        }
                    }
                }

                updateDisplay(drone);

                gpsGridPositionValue = drone.droneGPSListPosition;
                if (drone.droneControlStatus == "Docked Idle")
                {
                    drone.droneReady = true;
                }
                if (drone.droneControlStatus.Contains("Recharging") || drone.droneControlStatus.Contains("Unloading"))
                {
                    drone.droneReady = false;
                }
                if (drone.droneControlStatus.Contains("Idle") && drone.droneUndocked == "True" && drone.droneDocked == "False" && boresRemaining == 0 && !drone.droneRecallList)
                {
                    drone.droneRecallList = true;
                }
                if (drone.droneGPSListPosition == -1 && drone.droneAssignedCoordinates && drone.droneDocked == "True" && drone.droneControlStatus.Contains("Docked"))
                {
                    drone.droneAssignedCoordinates = false;
                }
                if (drone.droneGPSListPosition > -1) // attempting to reset droneGPSListPosition here if bore is finished - attempt here
                {
                    if (drone.droneControlSequence == 0 && drone.droneReady && drone.droneTunnelFinished == "False" && drone.droneControlStatus.Contains("Docked") && gridBoreFinished[drone.droneGPSListPosition] && drone.droneMining && drone.droneAssignedCoordinates && canRun)
                    {
                        drone.droneMining = false;
                        drone.droneAssignedCoordinates = false;
                        gridBoreOccupied[drone.droneGPSListPosition] = true; // cant guarantee this is occupied, might be occupied by another grid and not reported finished yet
                        drone.droneGPSListPosition = -1;
                        gpsGridPositionValue = -1;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();
                        if (canTransmit && drone.droneTransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.droneTransmissionStatus = false;
                        }
                    }
                    if (drone.droneControlSequence > 0 && drone.droneDocked == "True" && drone.droneControlStatus.Contains("Docked") && !drone.droneAssignedCoordinates && drone.droneGPSListPosition > -1 && canRun)
                    {
                        drone.droneMining = false;
                        drone.droneControlSequence = 0; //reset sequence if docked and not assigned
                        drone.droneGPSListPosition = -1;
                        gpsGridPositionValue = -1;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();
                        if (canTransmit && drone.droneTransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.droneTransmissionStatus = false;
                        }
                    }

                }
                Echo("herro1");
                if (drone.droneReady && drone.droneTunnelFinished == "False" && drone.droneDocked == "True" && canRun && !drone.droneAssignedCoordinates && drone.droneControlSequence == 0 && !drone.droneMustWait && !drone.droneMining && !disableRunArgument)
                {
                    Echo("herro1a");
                    if (gridBoresCompleted < totalMiningRuns && miningGridValid && !drone.droneAssignedCoordinates && !drone.droneMustWait)
                    {
                        Echo("herro1b");
                        if (gridBoreFinished.Count > 0)
                        {
                            Echo("herro1c");
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
                                if (gridBoreFinished.Count == 0)
                                {
                                    k = 0;
                                    break;
                                }
                                if (k > gridBoreFinished.Count - 1)
                                {
                                    k = gridBoreFinished.Count - 1;

                                }
                                if (!gridBoreFinished[k])
                                {
                                    if (droneRegistry.Count > 0)
                                    {
                                        Echo("trying");
                                        int queued_count = CountIntegerValues(droneRegistry, k);
                                        Echo($"maybe {queued_count}");

                                            if (!gridBoreOccupied[k] && queued_count > 0) //check if preassigned here
                                            {
                                                gridBoreOccupied[k] = true;
                                            }                                       
                                    }
                                }
                                Echo("whirling");
                                if (!gridBoreFinished[k] && !gridBoreOccupied[k])
                                    {
                                        currentGPSIndex = k;
                                        break;
                                    }                                
                            }
                        }
                        Echo("herro2");
                        realGPSIndex = currentGPSIndex;
                        if (gpsGridPositionValue == -1)
                        {
                            gpsGridPositionValue = currentGPSIndex;
                            drone.droneGPSCoordinates = gridBorePosition[gpsGridPositionValue];
                            drone.droneGPSListPosition = gpsGridPositionValue;
                        }
                        else
                        {
                            gpsGridPositionValue = drone.droneGPSListPosition;
                            if (gpsGridPositionValue > -1 && gpsGridPositionValue < gridBorePosition.Count)
                            {
                                drone.droneGPSCoordinates = gridBorePosition[gpsGridPositionValue];
                            }
                        }
                        if (!miningGridValid)
                        {
                            totalMiningRuns = 1;
                            if (droneGPSCoordinates.Count > 0)
                            {
                                drone.droneGPSCoordinates = miningGPSCoordinates;
                            }
                            gpsGridPositionValue = 0;
                            currentGPSIndex = 0;
                        }
                        //suspect code here
                        Echo($"Drone coords: {drone.droneName}");
                        if (drone.droneGPSListPosition > -1)
                        {
                            drone.droneAssignedCoordinates = true;
                            Echo($"Drone coords assigned: {drone.droneName} {drone.droneAssignedCoordinates}");
                        }
                        Echo("herro3");
                    }
                    else if (!miningGridValid)
                    {
                        totalMiningRuns = 1;
                        drone.droneGPSCoordinates = miningGPSCoordinates;
                        drone.droneAssignedCoordinates = true;
                        gpsGridPositionValue = 0;
                        currentGPSIndex = 0;
                        Echo("invalid grid - defaulting");
                    }
                    Echo("data staging");
                    if (drone.droneGPSListPosition > -1)
                    {
                        if (gridBoreOccupied[drone.droneGPSListPosition] && !drone.droneMining)
                        {
                            drone.droneMustWait = true;
                        }
                        else if ((totalDronesMining) < boresRemaining && gridBoresCompleted < totalMiningRuns || !gridBoreOccupied[drone.droneGPSListPosition] && !gridBoreFinished[drone.droneGPSListPosition] && !drone.droneMining)
                        {
                            drone.droneMustWait = false;
                        }
                        if (gridBoresCompleted != totalMiningRuns && !drone.droneMustWait)
                        {
                            drone.droneControlSequence = 1;
                            drone.droneMining = true;
                            gridBoreOccupied[drone.droneGPSListPosition] = true;
                        }
                        else
                        {
                            drone.droneControlSequence = 0;
                            drone.droneMining = false;
                        }

                        if (gridBoreFinished[drone.droneGPSListPosition])
                        {
                            //suspect coordinates here 2
                            Echo($"Drone position finished {drone.droneName}");
                            drone.droneControlSequence = 0;
                            drone.droneMining = false;
                            drone.droneAssignedCoordinates = false;
                            drone.droneGPSListPosition = -1;
                            gpsGridPositionValue = -1;
                        }
                    }
                }


                tx_chan = drone.droneName;
                cd1 = gpsGridPositionValue.ToString();
                cm = "0";
                xp = Math.Round(drone.droneGPSCoordinates.X, 2).ToString();
                yp = Math.Round(drone.droneGPSCoordinates.Y, 2).ToString();
                zp = Math.Round(drone.droneGPSCoordinates.Z, 2).ToString();
                cd5 = customData5;
                cd6 = (drillLength + safe_dstvl).ToString();
                igd = (ignoreDepth + safe_dstvl + drone_length - drone_clear_offset).ToString();
                if (prospectAlignTargetValid || customDataAlignTargetValid)
                {
                    xp2 = Math.Round(((drone.droneGPSCoordinates.X - miningGPSCoordinates.X) + alignGPSCoordinates.X), 2).ToString();
                    yp2 = Math.Round(((drone.droneGPSCoordinates.Y - miningGPSCoordinates.Y) + alignGPSCoordinates.Y), 2).ToString();
                    zp2 = Math.Round(((drone.droneGPSCoordinates.Z - miningGPSCoordinates.Z) + alignGPSCoordinates.Z), 2).ToString();
                }
                else
                {
                    xp2 = "";
                    yp2 = "";
                    zp2 = "";
                }
                if (drone.droneControlSequence == 1 && drone.droneAssignedCoordinates && !drone.droneMustWait && !disableRunArgument || drone.droneControlSequence == 2 && drone.droneControlStatus == "Docked Idle" && drone.droneDocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 2;
                    drone.droneMining = true;
                    if (drone.droneGPSListPosition > -1)
                    {
                        gridBoreOccupied[drone.droneGPSListPosition] = true;
                    }
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "7";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 2 && drone.droneControlStatus == "Undocked" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument || drone.droneControlSequence == 2 && drone.droneControlStatus == "Docking" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 3;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 2 && drone.droneControlStatus == "Undocking" && drone.droneDocked == "False" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument && drone.dcs <= bclu)
                {
                    drone.droneControlSequence = 13;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 8 && drone.droneControlStatus.Contains("RTB Ready") && drone.droneDocked == "False" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    if (drone.droneGPSListPosition > -1)
                    {
                        gridBoreOccupied[drone.droneGPSListPosition] = false; //trying to do this for mining efficiency - clear occupation when safely exited

                    }
                    gpsGridPositionValue = -1; //unassign drone.droneGPSListPosition from drone here if finished mining and in safe position

                    drone.droneControlSequence = 13;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 13 && drone.droneControlStatus == "Idle" && drone.droneDocked == "False" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument || drone.droneControlSequence == 5 && drone.droneControlStatus == "Docking" && drone.droneDocked == "False" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument && drone.dcs <= bclu)
                {
                    drone.droneControlSequence = 8;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "6";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 3 && drone.droneControlStatus == "Idle" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 4;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "4";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 4 && drone.droneControlStatus == "Nav End" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 5;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 4 && drone.droneControlStatus == "Docked Idle" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 5 && drone.droneControlStatus == "Idle" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 6;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "2";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 6 && drone.droneControlStatus == "Nav End" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 7;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 7 && drone.droneControlStatus == "Idle" && drone.droneUndocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 8;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "5";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence >= 8 && drone.droneControlStatus.Contains("Dock") && drone.droneMining || drone.droneControlSequence == 4 && drone.droneControlStatus.Contains("Docked") && drone.droneMining)
                {
                    if (drone.droneGPSListPosition > -1)
                    {
                        gridBoreOccupied[drone.droneGPSListPosition] = false;
                    }
                }

                if (drone.droneControlSequence >= 8 && (drone.droneControlStatus.Contains("Dock") || drone.droneControlStatus.Contains("Exit") || drone.droneControlStatus.Contains("RTB")) && drone.droneMining && drone.droneAssignedCoordinates && drone.droneMining && drone.droneTunnelFinished == "True")
                {
                    if (drone.droneGPSListPosition > -1)
                    {
                        if (!gridBoreFinished[drone.droneGPSListPosition])
                        {
                            gridBoreFinished[drone.droneGPSListPosition] = true; //Finish bore here
                            Echo($"Grid bore finished: {drone.droneGPSListPosition}");
                        }
                    }
                }


                if (drone.droneControlSequence == 8 && drone.droneReady && drone.droneDocked == "True" && (drone.droneTunnelFinished == "False" && drone.droneGPSListPosition > -1 && drone.droneGPSListPosition < gridBoreFinished.Count) && drone.droneAssignedCoordinates && !disableRunArgument)
                {
                    drone.droneControlSequence = 1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 8 && !drone.droneReady && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && drone.droneAssignedCoordinates && !disableRunArgument || drone.droneControlSequence == 8 && !drone.droneReady && drone.droneDocked == "True" && drone.droneTunnelFinished == "True" && drone.droneAssignedCoordinates && !disableRunArgument || drone.droneControlSequence >= 1 && drone.droneControlSequence <= 4 && !drone.droneReady && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && drone.droneAssignedCoordinates && !disableRunArgument)
                {
                    drone.droneControlSequence = 0;
                    drone.droneAssignedCoordinates = false;
                    drone.droneMining = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 8 && drone.droneReady && drone.droneMining && drone.droneDocked == "True" && (drone.droneTunnelFinished == "True") && drone.droneAssignedCoordinates && !disableRunArgument)
                {
                    drone.droneControlSequence = 9;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }



                if (drone.droneControlSequence == 8 && drone.droneReady && drone.droneDocked == "True" && drone.droneGPSListPosition == -1 && !drone.droneAssignedCoordinates && !disableRunArgument)
                {
                    drone.droneControlSequence = 0;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 9 && drone.droneReady && drone.droneDocked == "True" && (drone.droneTunnelFinished == "True") && canRun && drone.droneAssignedCoordinates && !disableRunArgument || drone.droneControlSequence == 9 && drone.droneReady && drone.droneDocked == "True" && (drone.droneTunnelFinished == "True") && (!drone.droneAssignedCoordinates) && !disableRunArgument)
                {
                    drone.droneControlSequence = 10;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }


                if (drone.droneControlSequence == 10 && drone.droneReady && drone.droneDocked == "True" && (drone.droneTunnelFinished == "True") && generalReset && drone.droneAssignedCoordinates && !disableRunArgument || drone.droneControlSequence == 10 && drone.droneReady && drone.droneDocked == "True" && (drone.droneTunnelFinished == "True") && drone.droneAssignedCoordinates && !disableRunArgument || drone.droneControlSequence == 0 && drone.droneReady && drone.droneDocked == "True" && (drone.droneTunnelFinished == "True") && drone.droneAssignedCoordinates && !disableRunArgument)
                {
                    drone.droneControlSequence = 11;
                    totalMiningSequencesComplete++;
                    gpsGridPositionValue = -1;
                    drone.droneResetFunction = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "8";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }


                if (drone.droneControlSequence == 11 && drone.droneReady && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && drone.droneAssignedCoordinates && totalMiningSequencesComplete <= totalMiningRuns && miningGridValid && !disableRunArgument)
                {
                    drone.droneControlSequence = 0;
                    drone.droneAssignedCoordinates = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlSequence == 11 && drone.droneControlStatus.Contains("Docked") && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && currentGPSIndex < totalMiningRuns && drone.droneAssignedCoordinates && totalMiningSequencesComplete > totalMiningRuns && !disableRunArgument || drone.droneControlSequence == 11 && drone.droneReady && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && drone.droneAssignedCoordinates && miningGridValid == false && totalMiningSequencesComplete >= totalMiningRuns && !disableRunArgument)
                {
                    drone.droneControlSequence = 12;
                    drone.droneAssignedCoordinates = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }

                    displayTextMain.Append('\n');
                    displayTextMain.Append("Mining seq. complete");
                }
                if (drone.droneControlSequence == 12 && drone.droneControlStatus.Contains("RTB") && drone.droneDocked == "False" && drone.droneTunnelFinished == "True" && !disableRunArgument)
                {
                    drone.droneAssignedCoordinates = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }
                if (drone.droneControlSequence == 12 && drone.droneControlStatus.Contains("Idle") && drone.droneDocked == "False" && !drone.droneAssignedCoordinates && !disableRunArgument)
                {
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "6";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlStatus.Contains("Docked") && drone.droneDocked == "True" && drone.droneTunnelFinished == "True" && generalReset || drone.droneControlStatus.Contains("Docked") && drone.droneDocked == "True" && drone.droneTunnelFinished == "True" && generalReset && !disableRunArgument)
                {
                    drone.droneControlSequence = 0;
                    totalMiningSequencesComplete = 0;
                    drone.droneAssignedCoordinates = false;
                    drone.droneMining = false;
                    currentGPSIndex = 0;
                    gpsGridPositionValue = -1;
                    drone.droneResetFunction = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "8";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (drone.droneControlStatus.Contains("Docked") && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && generalReset && drone.droneControlSequence == 0 && !disableRunArgument || drone.droneControlStatus.Contains("Docked") && drone.droneDocked == "True" && drone.droneTunnelFinished == "False" && generalReset && !disableRunArgument || drone.droneControlSequence == 6 && drone.droneControlStatus == "Docked Idle" && drone.droneDocked == "True" && drone.droneAssignedCoordinates && drone.droneMining && !disableRunArgument)
                {
                    drone.droneControlSequence = 0;
                    totalMiningSequencesComplete = 0;
                    drone.droneAssignedCoordinates = false;
                    drone.droneMining = false;
                    currentGPSIndex = 0;
                    gpsGridPositionValue = -1;
                    drone.droneResetFunction = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    drone.droneTranmissionOutput = c.ToString();
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (mustRecall_Command && !drone.droneRecallList && !mustUndockCommand)
                {
                    drone.droneRecallList = true;
                }
                if (drone.droneRecallList)
                {
                    if (drone.droneRecallSequence == 0 && drone.droneControlStatus == "Idle" || drone.droneRecallSequence == 0 && drone.droneControlStatus == "Undocked" || drone.droneRecallSequence == 0 && drone.droneControlStatus == "Nav" || drone.droneRecallSequence == 0 && drone.droneControlStatus == "Undocking" || drone.droneRecallSequence == 0 && drone.droneControlStatus == "Docking" || drone.droneRecallSequence == 0 && drone.droneControlStatus == "Initiating mining" || drone.droneRecallSequence == 0 && drone.droneControlStatus.Contains("RTB"))
                    {
                        drone.droneRecallSequence = 1;
                        if (drone.droneControlSequence > 0)
                        {
                            drone.droneControlSequence = 0;
                        }
                    }

                    if (drone.droneRecallSequence == 0 && drone.droneControlStatus == "Nav End")
                    {
                        drone.droneRecallSequence = 3;
                    }
                    if (drone.droneRecallSequence == 1)
                    {
                        drone.droneRecallSequence = 2;
                        drone.droneControlSequence = 0;
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();
                    }
                    if (drone.droneRecallSequence == 2 && drone.droneControlStatus == "Idle")
                    {
                        drone.droneRecallSequence = 3;
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "1";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();

                    }
                    if (drone.droneRecallSequence == 3 && drone.droneControlStatus == "Nav End")
                    {
                        drone.droneRecallSequence = 4;
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();

                    }
                    if (drone.droneRecallSequence == 3 && drone.droneControlStatus == "Nav" && drone.droneGPSListPosition == -1 || drone.droneRecallSequence == 3 && drone.droneControlStatus == "Idle" && drone.droneGPSListPosition >= -1)
                    {
                        drone.droneRecallSequence = 4;
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();

                    }

                    if (drone.droneRecallSequence == 4 && drone.droneControlStatus == "Idle")
                    {
                        drone.droneRecallSequence = 5;
                        drone.droneControlSequence = 0;
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "6";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();

                    }
                    if (drone.droneRecallSequence == 4 && drone.droneControlStatus == "Idle")
                    {
                        drone.droneRecallSequence = 5;
                        drone.droneControlSequence = 0;
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "6";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();

                    }
                    if (drone.droneRecallSequence == 5 && drone.droneControlStatus.Contains("Docked") || drone.droneRecallSequence == 0 && drone.droneControlStatus.Contains("Docked"))
                    {
                        drone.droneRecallSequence = 0;
                        drone.droneAssignedCoordinates = false;
                        drone.droneControlSequence = 0;
                        drone.droneRecallList = false;
                        drone.droneMining = false;
                        gpsGridPositionValue = -1;
                        drone.droneResetFunction = true;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();

                    }
                    if (canTransmit && drone.droneTransmissionStatus)
                    {
                        transmitToDrone(drone);
                        drone.droneTransmissionStatus = false;
                    }
                }

                if (mustUndockCommand)
                {
                    if (drone.droneControlStatus == "Docked Idle")
                    {
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "7";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();
                        if (canTransmit && drone.droneTransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.droneTransmissionStatus = false;
                        }
                    }
                }

                if (mustFreeze_Command)
                {
                    if (drone.droneControlStatus == "Undocked" || drone.droneControlStatus == "Idle")
                    {
                        gpsGridPositionValue = drone.droneGPSListPosition;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        drone.droneTranmissionOutput = c.ToString();
                        if (canTransmit && drone.droneTransmissionStatus)
                        {
                            transmitToDrone(drone);
                            drone.droneTransmissionStatus = false;
                        }
                    }
                }
                droneMessageConfirmed = false;
                receivedDroneName = "";


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
            if (droneRegistry.Count > 0)
            {
                maxActiveDronesCount = droneRegistry.Count - dronesInFlightFactor;
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

        private void ProcessJobGrid()
        {
            if (pbInterfaceActual == null || interfacePBTag[0] == null)
            {
                Echo($"Interface PB not found {interfaceTag.Replace("[", "[[").Replace("]", "]]")}");
            }
            #region job_grid_processing
            //if mining grid data empty resolve issues to avoid exception
            if (numPointsY == 0 && !gridCreated || numPointsX == 0 && !gridCreated || gridSize == 0 && !gridCreated)
            {
                gridBorePosition = new List<Vector3D>();
                gridBoreOccupied = new List<bool>();
                gridBoreFinished = new List<bool>();
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
            }
            if (!gridCreated)
            {

                if (!bores_regen)
                {
                    gridBorePosition = new List<Vector3D>();
                    gridBoreFinished = new List<bool>();
                    gridBoreOccupied = new List<bool>();
                    bores_regen = true;
                }

                if (readyFlag)
                {
                    readyFlag = false;
                }

                if (remoteControlTag[0] == null || remoteControlActual == null)
                {
                    Echo($"Remote control {antennaTagName.Replace("[", "[[").Replace("]", "]]")} not present - early exit");
                    return;
                }
                Vector3D gravity = remoteControlActual.GetNaturalGravity();

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

                if (!string.IsNullOrEmpty(Storage) && !string.IsNullOrWhiteSpace(Storage) && !gridCreated && bores_regen && !gridInitialisationComplete)
                {
                    //added from init
                    currentGPSIndex = 0;
                    realGPSIndex = currentGPSIndex;
                    GetStoredData(Storage);
                    Echo("Grid positions restored");
                    canLoading = true;
                    Storage = null;
                    //reset everything else
                    reset_drone_data();
                    reset_drone_list();
                    dronesPinged = false;
                    dronePingTimerCount = 0;
                    gridInitialisationComplete = true;
                }
                //coroutine management grid creation
                if (gridCoroutine == null && !gridInitialisationComplete && bores_regen || gridCoroutine != null && !gridCoroutine.MoveNext() && !gridInitialisationComplete && bores_regen)
                {
                    gridCoroutine = GenGrdPosits(centerPoint, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid);
                }
                if (gridCoroutine != null && !gridInitialisationComplete && bores_regen)
                {
                    // Check the current yield value
                    bool currentYield = gridCoroutine.Current;

                    // If the coroutine is finished, you can perform completion logic
                    if (!gridCoroutine.MoveNext())
                    {
                        // The coroutine has finished executing
                        Echo("Grid generation complete.");
                        gridCoroutine = null; // Reset the coroutine
                        GenGrdPosits(centerPoint, planeNrml, gridSize, numPointsX, numPointsY, coreOutGrid).Dispose();
                    }
                    else
                    {
                        // Handle intermediate status if needed
                        if (!currentYield)
                        {
                            Echo($"Generating grid positions... {Math.Round(percent_grid, 1)}%");
                            gridCoroutine.MoveNext();

                        }
                        if (currentYield)
                        {
                            //debugcount++;
                            gridInitialisationComplete = true;
                            pbInterfaceActual.CustomData = "";
                            canInit = false;
                            i_init = false;
                            interfaceArgument = "";
                        }
                    }

                }
                //grid data found - terminite initialisation
                if (gridBorePosition.Count > 0 && gridInitialisationComplete)
                {
                    gridCreated = true;
                    pbInterfaceActual.CustomData = "";
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
            Echo($"Grid: {gridCreated} - Bores: {totalMiningRuns} - Remaining: {boresRemaining}");
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
                if (droneMessagesBuffer.Count < droneRegistry.Count)
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
                if (droneMessagesBuffer.Count > droneRegistry.Count)
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
                        Echo($"Remote control {antennaTagName.Replace("[", "[[").Replace("]", "]]")} not present");
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
                if (droneRegistry.Count == 0 && !dronesPinged || droneRegistry.Count > 0 && !dronesPinged)
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
            string _currentCustomData = Me.CustomData;

            if (!string.IsNullOrEmpty(_currentCustomData) && !string.IsNullOrWhiteSpace(_currentCustomData))
            {
                mainCustomDataValid = true;
            }
            else
            {
                Echo($"Job custom data invalid - initialising job data");
                mainCustomDataValid = false;
                if (_currentCustomData != "GPS:---:0:0:0:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:False")
                {
                    miningCoordinatesNew.Clear();
                    miningCoordinatesNew.Append($"GPS:---:0:0:0:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:False");
                    InvalidJobDataWrite(Me, miningCoordinatesNew.ToString());
                }
            }
            if (mainCustomDataValid)
            {
                //FetchJobData(Me.CustomData);
                if (_currentCustomData != _oldCustomData)
                {
                    GetCustomDataJobCommand(Me.CustomData, Me);
                    _oldCustomData = Me.CustomData;
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
                    Echo($"Interface programmable block not present {interfaceTag.Replace("[", "[[").Replace("]", "]]")}");
                    return;
                }
                canInterfaceCommand = true;
                interfaceArgument = pbInterfaceActual.CustomData;
                Echo($"Interface PB: {interfaceTag.Replace("[", "[[").Replace("]", "]]")}");
                Echo($"Display command: {interfaceArgument} P:{prospectAlignTargetValid} C:{customDataAlignTargetValid}");
            }
            #endregion
            #region interface_command_processing
            if (canInterfaceCommand && pbInterfaceActual.CustomData != null)
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
            #endregion
        }

        private void HandleCommands(string argument)
        {
            #region run_command_processing
            if (argument == "setup" && setupComplete)
            {
                setupComplete = false;
                argument = "";
                Echo("Running Setup..");
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

        private void DroneRenderCall()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (display_tag_drone.Count == 0 || droneRegistry.Count == 0 || display_tag_drone[0] == null) return;

            // Convert Dictionary to a List once to ensure a stable "snapshot" for this frame
            var drones = droneRegistry.Values.ToList();

            if (renew_header)
            {
                droneInformation.Clear().Append($"Mining Drone Status {secondary_tag} [{drone_tag}] - GMDC {ver} {icon}\n");
                renew_header = false;
            }

            int dronesPerDisplay = drones_per_screen * display_tag_drone.Count;
            if (dronesPerDisplay < drones.Count)
            {
                Echo($"Insufficient displays '{dp_drn_tag.Replace("[", "[[").Replace("]", "]]")}': {dronesPerDisplay} < {drones.Count}");
                return;
            }

            // We increment by 2 because DroneScreenBuilder handles a "pair" (Left and Right column)
            for (int i = 0; i < drones.Count; i += 2)
            {
                Drone d1 = drones[i];
                Drone d2 = (i + 1 < drones.Count) ? drones[i + 1] : null;
                bool hasPair = (d2 != null);

                // Call your new refactored Builder that takes Drone objects
                DroneScreenBuilder(d1, d2, hasPair);

                // Calculate which LCD this pair belongs to
                int displayIndex = i / drones_per_screen;

                // Determine if it's time to print this page to the LCD 
                // (Either the screen is full, or we reached the end of the drone list)
                if (displayIndex < display_tag_drone.Count && display_tag_drone[displayIndex] != null)
                {
                    if ((i + 2) % drones_per_screen == 0 || (i + 1) >= drones.Count - 1)
                    {
                        sD = ((IMyTextSurfaceProvider)display_tag_drone[displayIndex]).GetSurface(srfD);
                        sD.WriteText(droneInformation);
                        renew_header = true;
                    }
                }
            }
        }
        private struct DroneStats
        {
            public int Docking, Docked, Undocking, Undocked, Damage, Unknown, Ok, Exit, Idle, Recharge, Unload, Mining, RTBA, RTBB, Nav, IdleD;
        }

        private void UpdateDroneCounts()
        {
            // Reset our stats object to zero
            DroneStats stats = new DroneStats();

            // reset total counts that we calculate inside this loop
            totalDronesActive = 0;

            // This is how you loop through a Dictionary
            foreach (var kvp in droneRegistry)
            {
                // kvp is a KeyValuePair. .Value is your Drone object.
                Drone drone = kvp.Value;

                string status = drone.droneControlStatus;
                string damage = drone.droneDamageState; // Note: using the property from the object now!

                // Basic Active check
                if (drone.droneMining) totalDronesActive++;

                // Status Counts
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

                // Damage Counts
                stats.Damage += (damage == "DMG") ? 1 : 0;
                stats.Unknown += (damage == "UNK") ? 1 : 0;
                stats.Ok += (damage == "OK") ? 1 : 0;

                // Custom check for undocking sequence (if you still need total_drones_undocking)
                if (drone.droneControlSequence == 2) total_drones_undocking++;
            }

            // Apply stats to your global variables
            t_drn_dckg = stats.Docking;
            t_drn_dck = stats.Docked;
            t_drn_udckg = stats.Undocking;
            t_drn_udck = stats.Undocked;
            t_drn_exit = stats.Exit;
            t_drn_idle_undocked = stats.Idle;
            t_drn_rechg = stats.Recharge;
            t_drn_unload = stats.Unload;
            t_drn_mine = stats.Mining;
            t_drn_nav = stats.Nav;
            t_drn_idle_docked = stats.IdleD;

            totalDronesDamaged = stats.Damage;
            totalDronesUnknown = stats.Unknown;

            // Logic for mining count
            totalDronesMining = Math.Max(0, totalDronesActive - t_drn_dckg);

            // Bore calculations (Assuming these are still in global lists for now)
            gridBoresCompleted = CountTrueValues(gridBoreFinished);
            boresRemaining = totalMiningRuns - gridBoresCompleted;
        }
        private void UpdateDroneCountsOld(Drone drone)
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            gridBoresCompleted = CountTrueValues(gridBoreFinished);
            boresRemaining = totalMiningRuns - gridBoresCompleted;
            totalDronesActive = CountTrueValues(droneMining);
            //total_drones_undocking = CountIntegerValues(droneControlSequence, 2);

            DroneStats stats = new DroneStats();
            for (int i = 0; i < droneRegistry.Count; i++)
            {
                string status = drone.droneControlStatus;
                string damage = drone.droneDamageState;
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
            totalDronesDamaged = stats.Damage; totalDronesUnknown = stats.Unknown; //t_dn_ok = stats.Ok;
            totalDronesMining = totalDronesActive - t_drn_dckg;
            if (totalDronesMining < 0)
            {
                totalDronesMining = 0;
            }
            //Echo($"UpdateDroneCounts: {Runtime.CurrentInstructionCount - startInstructions}");
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

                for (int d = 0; d < droneGPSListPosition.Count; d++)
                    if (droneGPSListPosition[d] >= 0 && droneGPSListPosition[d] < boreQueueCounts.Length)
                        boreQueueCounts[droneGPSListPosition[d]]++;

                for (int l = 0; l < gridBoreOccupied.Count; l++)
                    if (gridBoreOccupied[l] && boreQueueCounts[l] == 0)
                        gridBoreOccupied[l] = false;
            }
            //Echo($"DroneUndockCheck: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        public void updateDisplay(Drone drone)
        {            
            displayTextMain.Append($"Drone Controller Status - GMDC {ver} - [{drone_tag}] {icon}");
            displayTextMain.Append('\n');
            if (drone.droneControlSequence == 12)
            {
                gpsGridPositionValue = -1;
                drone.droneMining = false;
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
                drone.droneControlSequence = 12;
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

       
        
        public void GetRemoteControlData(string input, IMyTerminalBlock block)
        {
            // 1. Early exits and caching
            if (block == null || remoteControlTag[0] == null)
            {
                // Only run Replace if we actually need to Echo (saves instructions)
                Echo($"Remote Control {antennaTagName.Replace("[", "[[").Replace("]", "]]")} not present");
                return;
            }

            string rawData = block.CustomData; // Cache once

            if (string.IsNullOrWhiteSpace(rawData))
            {
                Echo("Prospector job data not found");
                return;
            }

            // 2. Initial Tag Check
            if (!rawData.Contains("[GMDCJobData]"))
            {
                // Don't split unless we have to
                if (rawData.Length > 0)
                {
                    StoreRawInput(rawData, block, gmdccategory, rcjobinfo);
                }
                return;
            }

            // Reuse the previous logic for input validation
            if (string.IsNullOrWhiteSpace(input)) return;

            FetchRCJobData(rawData);
            string[] remoteGpsCommand = rcjobdata.Split(':');
            int len = remoteGpsCommand.Length;

            // 3. Reset State
            prospectTargetValid = false;

            if (len < 6)
            {
                //ClearRCCustomData1_6(1, 6); // Helper to clear variables in bulk
                return;
            }

            // 4. Primary Target Parsing
            // Using the ParseDouble helper we created earlier
            targetGPSCoordinates.X = ParseDouble(remoteGpsCommand, 2, 0.0);
            targetGPSCoordinates.Y = ParseDouble(remoteGpsCommand, 3, 0.0);
            targetGPSCoordinates.Z = ParseDouble(remoteGpsCommand, 4, 0.0);
            safe_dstvl = ParseDouble(remoteGpsCommand, 6, 0.0);

            // If we have valid X coordinate, assume target is valid (common SE shortcut)
            prospectTargetValid = (targetGPSCoordinates.X != 0);

            // Bulk assign the strings if they are needed elsewhere

            // 5. Alignment Target Parsing
            if (len > 7 && !prospectAlignTargetValid)
            {
                if (len < 11)
                {
                    //ClearRCCustomData7_12(7, 12);
                    prospectAlignTargetValid = false;
                }
                else
                {
                    alignGPSCoordinates.X = ParseDouble(remoteGpsCommand, 9, 0.0);
                    alignGPSCoordinates.Y = ParseDouble(remoteGpsCommand, 10, 0.0);
                    alignGPSCoordinates.Z = ParseDouble(remoteGpsCommand, 11, 0.0);

                    // Validation check
                    prospectAlignTargetValid = (alignGPSCoordinates.X != 0 || alignGPSCoordinates.Y != 0);

                    StoreRCJobData(remoteControlActual, rcjobdata);
                }
            }
        }

        void FetchRCJobData(string input)
        {
            _customDataStore.Clear();
            if (_customDataStore.TryParse(input))
            {
                var str = "";
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
                    if (!double.TryParse(vectorsplita[3], out alignGPSCoordinates.Y))
                    {
                        alignGPSCoordinates.Y = 0.0;
                    }
                    if (!double.TryParse(vectorsplita[4], out alignGPSCoordinates.Z))
                    {
                        alignGPSCoordinates.Z = 0.0;
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
            _customDataStore.Clear();
        }
        void StoreRCJobData(IMyTerminalBlock block, string input)
        {
            _customDataStore.Clear();
            _customDataStore.Set(gmdccategory, rcjobinfo, input);
            _customDataStore.Set(gmdccategory, "TargetGPS", $"GPS:PDT:{targetGPSCoordinates.X}:{targetGPSCoordinates.Y}:{targetGPSCoordinates.Z}:#FF75C9F1:");
            _customDataStore.Set(gmdccategory, "AlignGPS", $"GPS:TGT:{alignGPSCoordinates.X}:{alignGPSCoordinates.Y}:{alignGPSCoordinates.Z}:#F77668:");
            _customDataStore.Set(gmdccategory, "SafeAlignDistance", safe_dstvl);
            block.CustomData = _customDataStore.ToString();
            _customDataStore.Clear();
        }


        public void GetCustomDataJobCommand(string input, IMyTerminalBlock block)
        {
            // 1. Cache CustomData once to avoid multiple API calls
            string rawData = block.CustomData;

            if (string.IsNullOrWhiteSpace(rawData))
            {
                Echo("Datablank");
                return;
            }

            // 2. Initial format check - avoids splitting if not needed
            if (!rawData.Contains(gmdccategory))
            {
                string[] testSplit = rawData.Split(':');
                if (testSplit.Length > 0)
                {
                    StoreRawInput(rawData, block, gmdccategory, jobinfo);
                }
                Echo("Dataconversion");
                return;
            }

            FetchJobData(rawData);
            string[] gpsCommand = jobdata.Split(':');
            int len = gpsCommand.Length;

            if (len < 10)
            {
                // Use a loop or clear method to reset variables instead of 22 lines of code
                ClearCustomDataVariables();
                Echo("Data format invalid - GPS:name:x:y:z...");
                return;
            }

            // 3. Batch processing using the helpers
            miningGPSCoordinates.X = ParseDouble(gpsCommand, 2, 0.0);
            miningGPSCoordinates.Y = ParseDouble(gpsCommand, 3, 0.0);
            miningGPSCoordinates.Z = ParseDouble(gpsCommand, 4, 0.0);
            customData5 = gpsCommand[5];
            // Set validation flag based on coordinate success
            miningCoordsValid = (miningGPSCoordinates.X != 0 || miningGPSCoordinates.Y != 0 || miningGPSCoordinates.Z != 0);

            drillLength = ParseDouble(gpsCommand, 6, 1.0);
            gridSize = ParseDouble(gpsCommand, 7, 0.0);
            numPointsX = ParseInt(gpsCommand, 8, 0);
            numPointsY = ParseInt(gpsCommand, 9, 0);
            ignoreDepth = ParseDouble(gpsCommand, 10, 0.0);

            bool.TryParse(gpsCommand.ElementAtOrDefault(11), out dronesLaunchedStatus);
            dronesInFlightFactor = ParseInt(gpsCommand, 12, 1);
            dronesActiveHardLimit = ParseInt(gpsCommand, 13, 6);
            skipBoresNumber = ParseInt(gpsCommand, 14, 0);
            bool.TryParse(gpsCommand.ElementAtOrDefault(15), out coreOutGrid);

            // 4. Alignment Logic
            customDataAlignTargetValid = false;
            if (len > 22)
            {
                alignGPSCoordinates.X = ParseDouble(gpsCommand, 18, 0.0);
                alignGPSCoordinates.Y = ParseDouble(gpsCommand, 19, 0.0);
                alignGPSCoordinates.Z = ParseDouble(gpsCommand, 20, 0.0);
                safe_dstvl = ParseDouble(gpsCommand, 22, 30.0);

                customDataAlignTargetValid = (alignGPSCoordinates.X != 0);
            }

            // 5. Optimized Update - Use StringBuilder instead of + concatenation
            if (prospectAlignTargetValid && len > 16 && len < 18)
            {
                _statusBuffer.Clear();
                _statusBuffer.Append(jobdata)
                             .Append("GPS:TGT:")
                             .Append(alignGPSCoordinates.X).Append(":")
                             .Append(alignGPSCoordinates.Y).Append(":")
                             .Append(alignGPSCoordinates.Z).Append(":#F77668:")
                             .Append(safe_dstvl).Append(":");

                StoreRawInput(_statusBuffer.ToString(), block, gmdccategory, jobinfo);
            }
        }
 
        void FetchJobData(string input)
        {
            _customDataStore.Clear();
            if (_customDataStore.TryParse(input))
            {
                var str = "";
                str = _customDataStore.Get(gmdccategory, jobinfo).ToString().Trim();
                jobdata = str;
                /*str = _customDataStore.Get(gmdccategory, "TargetGPS").ToString().Trim();
                String[] vectorsplit = str.Split(':');
                if (vectorsplit.Length >= 5)
                {
                    if (!double.TryParse(vectorsplit[2], out miningGPSCoordinates.X))
                    {
                        miningGPSCoordinates.X = 0.0;
                    }
                    if (!double.TryParse(vectorsplit[3], out miningGPSCoordinates.Y))
                    {
                        miningGPSCoordinates.Y = 0.0;
                    }
                    if (!double.TryParse(vectorsplit[4], out miningGPSCoordinates.Z))
                    {
                        miningGPSCoordinates.Z = 0.0;
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
                    if (!double.TryParse(vectorsplit[2], out alignGPSCoordinates.X))
                    {
                        alignGPSCoordinates.X = 0.0;
                    }
                    if (!double.TryParse(vectorsplit[3], out alignGPSCoordinates.Y))
                    {
                        alignGPSCoordinates.Y = 0.0;
                    }
                    if (!double.TryParse(vectorsplit[4], out alignGPSCoordinates.Z))
                    {
                        alignGPSCoordinates.Z = 0.0;
                    }
                }
                else
                {
                    alignGPSCoordinates.X = 0.0;
                    alignGPSCoordinates.Y = 0.0;
                    alignGPSCoordinates.Z = 0.0;
                }
                str = _customDataStore.Get(gmdccategory, "BoreSeparation").ToString().Trim();
                if (!double.TryParse(str, out gridSize))
                {
                    gridSize = 10.0;
                }
                str = _customDataStore.Get(gmdccategory, "GridXBores").ToString().Trim();
                if (!int.TryParse(str, out numPointsX))
                {
                    numPointsX = 1;
                }
                str = _customDataStore.Get(gmdccategory, "GridYBores").ToString().Trim();
                if (!int.TryParse(str, out numPointsY))
                {
                    numPointsY = 1;
                }
                str = _customDataStore.Get(gmdccategory, "SkipBores").ToString().Trim();
                if (!int.TryParse(str, out skipBoresNumber))
                {
                    skipBoresNumber = 0;
                }
                str = _customDataStore.Get(gmdccategory, "SafeAlignDistance").ToString().Trim();
                if (!double.TryParse(str, out safe_dstvl))
                {
                    safe_dstvl = 30.0;
                }
                str = _customDataStore.Get(gmdccategory, "DrillDepth").ToString().Trim();
                if (!double.TryParse(str, out drillLength))
                {
                    drillLength = 30.0;
                }
                str = _customDataStore.Get(gmdccategory, "IgnoreDepth").ToString().Trim();
                if (!double.TryParse(str, out ignoreDepth))
                {
                    ignoreDepth = 0.0;
                }
                str = _customDataStore.Get(gmdccategory, "LimitDronesInFlight").ToString().Trim();
                if (!bool.TryParse(str, out dronesLaunchedStatus))
                {
                    dronesLaunchedStatus = false;
                }
                str = _customDataStore.Get(gmdccategory, "DronesFlightHardLimit").ToString().Trim();
                if (!int.TryParse(str, out dronesActiveHardLimit))
                {
                    dronesActiveHardLimit = 10;
                }
                str = _customDataStore.Get(gmdccategory, "DronesFlightFactor").ToString().Trim();
                if (!int.TryParse(str, out dronesInFlightFactor))
                {
                    dronesInFlightFactor = 1;
                }
                str = _customDataStore.Get(gmdccategory, "CoreOutFunction").ToString().Trim();
                if (!bool.TryParse(str, out coreOutGrid))
                {
                    coreOutGrid = false;
                }   */
            }
            _customDataStore.Clear();
        }

        public void DroneScreenBuilder(Drone drone1, Drone drone2, bool slu)
        {
            // If drone1 is null, we can't draw anything
            if (drone1 == null) return;

            string butter = "N/A";
            string butter2 = "N/A";

            // Use drone1 properties instead of parallel lists
            if (drone1.droneGPSListPosition != -1 && gridBoreFinished.Count > drone1.droneGPSListPosition)
            {
                butter = gridBoreFinished[drone1.droneGPSListPosition].ToString();
            }

            // Build lines for the first drone
            cl[0] = $"{drone1.droneName} Status: {drone1.droneDamageState} {drone1.droneControlStatus}";
            cl[1] = $"{drone1.droneName} Docked: {drone1.droneDocked} Rdy: {drone1.droneReady}";
            cl[2] = $"{drone1.droneName} Undocked: {drone1.droneUndocked}";
            cl[3] = $"{drone1.droneName} Finished: {drone1.droneTunnelFinished} Bore: {butter}";
            cl[4] = $"{drone1.droneName} Mining: {drone1.droneMining}";
            cl[5] = $"{drone1.droneName} Waiting: {drone1.droneMustWait} Reset: {drone1.droneResetFunction}";
            cl[6] = $"Charge: {drone1.drone_charge_storage}% Tank: {drone1.drone_gas_storage}% Cargo: {drone1.drone_ore_storage}%";
            cl[7] = $"Drill depth: {drone1.droneBoreDepth}m Start: {drone1.drone_mine_depth_start_status}m";
            cl[8] = $"Current depth: {drone1.droneBoreDepthCurrent}m";
            cl[9] = $"Seq: {drone1.droneControlSequence} Recall: {drone1.droneRecallSequence} {drone1.droneRecallList}";
            cl[10] = $"Loc: {drone1.droneGPSListPosition} Asnd: {drone1.droneAssignedCoordinates} OK: {drone1.dst}";
            cl[11] = $"X: {drone1.drone_location_x} Y: {drone1.drone_location_y} Z: {drone1.drone_location_z}";

            if (slu && drone2 != null)
            {
                if (drone2.droneGPSListPosition != -1 && gridBoreFinished.Count > drone2.droneGPSListPosition)
                {
                    butter2 = gridBoreFinished[drone2.droneGPSListPosition].ToString();
                }

                cl2[0] = $"{drone2.droneName} Status: {drone2.droneDamageState} {drone2.droneControlStatus}";
                cl2[1] = $"{drone2.droneName} Docked: {drone2.droneDocked} Rdy: {drone2.droneReady}";
                cl2[2] = $"{drone2.droneName} Undocked: {drone2.droneUndocked}";
                cl2[3] = $"{drone2.droneName} Finished: {drone2.droneTunnelFinished} Bore: {butter2}";
                cl2[4] = $"{drone2.droneName} Mining: {drone2.droneMining}";
                cl2[5] = $"{drone2.droneName} Waiting: {drone2.droneMustWait} Reset: {drone2.droneResetFunction}";
                cl2[6] = $"Charge: {drone2.drone_charge_storage}% Tank: {drone2.drone_gas_storage}% Cargo: {drone2.drone_ore_storage}%";
                cl2[7] = $"Drill depth: {drone2.droneBoreDepth}m Start: {drone2.drone_mine_depth_start_status}m";
                cl2[8] = $"Current depth: {drone2.droneBoreDepthCurrent}m";
                cl2[9] = $"Seq: {drone2.droneControlSequence} Recall: {drone2.droneRecallSequence} {drone2.droneRecallList}";
                cl2[10] = $"Loc: {drone2.droneGPSListPosition} Asnd: {drone2.droneAssignedCoordinates} OK: {drone2.dst}";
                cl2[11] = $"X: {drone2.drone_location_x} Y: {drone2.drone_location_y} Z: {drone2.drone_location_z}";
            }

            // Rendering loop remains mostly the same, just ensure it handles the slu logic safely
            droneInformation.AppendLine();
            for (int i = 0; i < 12; i++)
            {
                int padding = Math.Max(clbs - cl[i].Length, 0);
                droneInformation.Append(cl[i]).Append(' ', padding);
                if (slu && drone2 != null) droneInformation.AppendLine(cl2[i]);
                else droneInformation.AppendLine();
            }
        }

        IEnumerator<bool> GenListDisplay()
        {
            if (!listHeaderGenerated)
            {
                displayTextList.Append($"{secondary_tag} Mining Grid Status - GMDC {ver} {icon}");
                displayTextList.Append('\n');
                displayTextList.Append('\n');
                displayTextList.Append($"Remaining bores: {boresRemaining} - Current Index: {currentGPSIndex}");
                displayTextList.Append('\n');
                listHeaderGenerated = true;
            }

            for (int i = 0; i < gridBoreFinished.Count; i++)
            {
                foreach (var kvp in droneRegistry)
                {
                    Drone drone = kvp.Value;
                    if (!gridBoreOccupied[i])
                    {
                        drone_namer = "";
                    }
                    else if (i == drone.droneGPSListPosition)
                    {
                        drone_namer = drone.droneName;
                        drone.drone_assigns_count++;
                    }
                    if (drone.drone_assigns_count > 1)
                    {
                        gridBoreOccupied[i] = false;
                    }
                    drone.drone_assigns_count = 0;

                }
                if (!gridBoreFinished[i])
                {
                    displayTextList.Append('\n');
                    displayTextList.Append($"Grid Index: {i} - Occuipied: {gridBoreOccupied[i]} - Assigned: {drone_namer}");

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

        IEnumerator<bool> GenGrdPosits(Vector3D centerPoint, Vector3D planeNormal, double gridSize, int numPointsX, int numPointsY, bool coreout)
        {
            //debugcount++;
            //initgridcount++;
            //List<Vector3D> grdPositins = new List<Vector3D>();

            int gridcount_inner = 0;
            int gridcount_outer = 0;
            int gridcount = 0;
            int core_numpoints_x = 0;
            int core_numpoints_y = 0;

            Vector3D xAxis = Vector3D.CalculatePerpendicularVector(planeNormal);
            Vector3D yAxis = Vector3D.Cross(planeNormal, xAxis);
            Vector3D halfOffsetX = (numPointsX - 1) * 0.5 * gridSize * xAxis;
            Vector3D halfOffsetY = (numPointsY - 1) * 0.5 * gridSize * yAxis;
            gridcount_outer = numPointsX * numPointsY;
            for (int i = 0; i < numPointsX; i++)
            {
                for (int j = 0; j < numPointsY; j++)
                {
                    Vector3D position = centerPoint + i * gridSize * xAxis - j * gridSize * yAxis - halfOffsetX + halfOffsetY;

                    gridBorePosition.Add(position);
                    gridBoreOccupied.Add(false);
                    gridBoreFinished.Add(false);
                }
                yield return false;
            }
            if (coreout)
            {
                core_numpoints_x = numPointsX - 1;
                core_numpoints_y = numPointsY - 1;
                Vector3D halfOffsetX_core = (core_numpoints_x - 1) * 0.5 * gridSize * xAxis;
                Vector3D halfOffsetY_core = (core_numpoints_y - 1) * 0.5 * gridSize * yAxis;
                if (core_numpoints_x < 1)
                {
                    core_numpoints_x = 1;
                }
                if (core_numpoints_y < 1)
                {
                    core_numpoints_y = 1;
                }
                gridcount_inner = core_numpoints_x * core_numpoints_y;
                if (gridcount_inner >= 1)
                {
                    for (int i = 0; i < core_numpoints_x; i++)
                    {
                        for (int j = 0; j < core_numpoints_y; j++)
                        {
                            Vector3D position = centerPoint + i * gridSize * xAxis - j * gridSize * yAxis - halfOffsetX_core + halfOffsetY_core;
                            gridBorePosition.Add(position);
                            gridBoreOccupied.Add(false);
                            gridBoreFinished.Add(false);
                        }
                        yield return false;
                    }
                }
            }
            gridcount = gridcount_inner + gridcount_outer;
            percent_grid = (double)gridBorePosition.Count / (double)gridcount;
            //Echo($"{gridcount} {grid_bore_positions.Count} {gridcount}");
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

        IEnumerator<bool> BuildSprites(Vector3D centerPoint, Vector3D planeNormal, double gridSize, int numPointsX, int numPointsY, bool coreout)
        {
            //sprites.Clear();
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
            //build sprite frame

            if (gridBorePosition.Count > 0)
            {
                //m_gps_crds,planeNrml, grdsz, nPtsX, nPtsY, core_out
                //planeNrml, grdsz, nPtsX, nPtsY, core_out
                var text_position = new Vector2(256, 20) + _viewport.Position;
                var spriteText = new MySprite()
                {
                    Type = SpriteType.TEXT,
                    Data = $"--- {secondary_tag} Mining Grid  Status ---",
                    Position = text_position,
                    RotationOrScale = 1.0f,
                    Size = sizer,
                    Color = Color.WhiteSmoke.Alpha(1.0f),
                    Alignment = TextAlignment.CENTER,
                    FontId = "White"
                };
                sprites.Add(spriteText);
                text_position = new Vector2(256, 60) + _viewport.Position;
                spriteText = new MySprite()
                {
                    Type = SpriteType.TEXT,
                    Data = $"Total Bores: {totalMiningRuns} - Remaining:{boresRemaining} - Drones: {totalDronesMining} ({totalDronesActive})",
                    Position = text_position,
                    RotationOrScale = 0.7f,
                    Size = sizer,
                    Color = Color.WhiteSmoke.Alpha(1.0f),
                    Alignment = TextAlignment.CENTER,
                    FontId = "White"
                };
                sprites.Add(spriteText);

                Vector3D xAxis = Vector3D.CalculatePerpendicularVector(planeNormal);
                Vector3D yAxis = Vector3D.Cross(planeNormal, xAxis);

                //homedirectionsprite calculation
                // Define the fixed radius (adjust this value as needed, e.g., in pixels)
                float screenRadius = Math.Min(_viewport.Size.X, _viewport.Size.Y) / 2 * 0.8f; // Example
                if (remoteControlActual != null)
                {
                    Vector3D LocalRCHome = remoteControlActual.GetPosition();
                    Vector3D relativePointHome = LocalRCHome - centerPoint;
                    string ImageHome;
                    ImageHome = "AH_BoreSight";
                    var home_colour = new Color();
                    var alpha_bytes_home = 1.0f;
                    home_colour = Color.Purple;
                    double xPlanarHome = Vector3D.Dot(relativePointHome, xAxis);
                    double yPlanarHome = Vector3D.Dot(relativePointHome, yAxis);
                    var CentXHome = (float)xPlanarHome * scale_factor_x;
                    var CentYHome = -(float)yPlanarHome * scale_factor_y;
                    // Calculate the magnitude of the direction vector
                    float mag = (float)Math.Sqrt(CentXHome * CentXHome + CentYHome * CentYHome);
                    // Normalize the direction
                    float normX = CentXHome / mag;
                    float normY = CentYHome / mag;

                    var positionHome = _viewport.Center + screenRadius * new Vector2(normX, normY);
                    //float rotationHome = (float)Math.Atan2(yPlanarHome, xPlanarHome);
                    float rotationHome = (float)Math.Atan2(CentYHome, CentXHome);
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
                for (int i = 0; i < gridBorePosition.Count; i++)
                {
                    sprite_total++;
                    Vector3D relativePoint = gridBorePosition[i] - centerPoint;
                    double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                    double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                    var CentX = (float)xPlanar;
                    var CentY = -(float)yPlanar;
                    string Image;
                    var bore_colour = new Color();
                    var alpha_bytes = 1.0f;
                    Image = gridBoreFinished[i] ? "CircleHollow" : "Circle";
                    alpha_bytes = gridBoreOccupied[i] ? 1.0f : 0.5f;
                    bore_colour = gridBoreOccupied[i] ? Color.LightSkyBlue : Color.DeepSkyBlue;

                    var position = new Vector2(CentX * scale_factor_x, CentY * scale_factor_y) + _viewport.Center;
                    //background sprite
                    var sprite = new MySprite()
                    {
                        Type = SpriteType.TEXTURE,
                        Data = Image,
                        Position = position,
                        //RotationOrScale = size_scale,
                        Size = sizer,
                        Color = bore_colour.Alpha(alpha_bytes),
                        Alignment = TextAlignment.CENTER
                    };
                    //Echo($"{position}");
                    sprites.Add(sprite);
                    percent_list_vis = (((double)i + (double)1) / ((double)gridBorePosition.Count)) * 100;
                    spriteCounter++;
                    yield return false;
                }

                if (droneRegistry.Count > 0)
                {
                    drone_total = 0;
                    foreach (var drone in droneRegistry.Values.ToList())
                    {
                        //Drone drone = kvp.Value;
                        double drone_locale_x = 0.0;
                        double drone_locale_y = 0.0;
                        double drone_locale_z = 0.0;
                        drone_total++;
                        if (!double.TryParse(drone.drone_location_x, out drone_locale_x))
                        {
                            drone_locale_x = 0.0;
                        }
                        if (!double.TryParse(drone.drone_location_y, out drone_locale_y))
                        {
                            drone_locale_y = 0.0;
                        }
                        if (!double.TryParse(drone.drone_location_z, out drone_locale_z))
                        {
                            drone_locale_z = 0.0;
                        }

                        Vector3D Drone_point = new Vector3D(drone_locale_x, drone_locale_y, drone_locale_z);

                        Vector3D relativePoint = Drone_point - centerPoint;
                        double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                        double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                        var CentX = (float)xPlanar;
                        var CentY = -(float)yPlanar;
                        string Image_drone = "";
                        var bore_colour_drone = new Color();
                        var alpha_val = 1.0f;

                        if (drone.droneControlStatus.Contains("Docked") || drone.droneControlStatus.Contains("Undocked") || drone.droneControlStatus.Contains("Docking") || drone.droneControlStatus.Contains("Undocking"))
                        {
                            alpha_val = 0.25f;
                        }
                        else
                        {
                            alpha_val = 1.0f;
                        }
                        if (!drone.droneMining)
                        {
                            Image_drone = "Circle";
                            bore_colour_drone = Color.Gray;
                        }
                        if (drone.droneMining)
                        {
                            Image_drone = "Circle";

                            if (drone.droneControlStatus.Contains("Min"))
                            {
                                bore_colour_drone = Color.Purple;
                            }
                            else if (drone.droneControlStatus.Contains("Exit"))
                            {
                                bore_colour_drone = Color.Orange;
                            }
                            else if (drone.droneControlStatus.Contains("RTB: Ready"))
                            {
                                bore_colour_drone = Color.Green;
                            }
                            else if (drone.droneControlStatus.Contains("Undock"))
                            {
                                bore_colour_drone = Color.Yellow;
                            }
                            else
                            {
                                bore_colour_drone = Color.Navy;
                            }
                            alpha_val = 1.0f;
                        }
                        if (drone.droneDamageState == "DMG")
                        {
                            Image_drone = "Circle";
                            bore_colour_drone = Color.Red;
                        }

                        var position = new Vector2(CentX * scale_factor_x, CentY * scale_factor_y) + _viewport.Center;
                        //background sprite
                        var sprite = new MySprite()
                        {
                            Type = SpriteType.TEXTURE,
                            Data = Image_drone,
                            Position = position,
                            //RotationOrScale = size_scale,
                            Size = sizer * 0.8f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER
                        };
                        sprites.Add(sprite);
                        if (drone.drone_cargo_full.Contains("True") || drone.drone_recharge_request.Contains("True"))
                        {
                            if (drone.drone_recharge_request.Contains("True") && drone.drone_cargo_full.Contains("True"))
                            {
                                bore_colour_drone = Color.White;
                            }
                            else if (drone.drone_recharge_request.Contains("True"))
                            {
                                bore_colour_drone = Color.YellowGreen;
                            }
                            else if (drone.drone_cargo_full.Contains("True"))
                            {
                                bore_colour_drone = Color.RosyBrown;
                            }
                            Image_drone = "CircleHollow";
                            var sprite_layer_h = new MySprite()
                            {
                                Type = SpriteType.TEXTURE,
                                Data = Image_drone,
                                Position = position,
                                //RotationOrScale = size_scale,
                                Size = sizer * 0.8f,
                                Color = bore_colour_drone.Alpha(alpha_val),
                                Alignment = TextAlignment.CENTER
                            };
                            sprites.Add(sprite_layer_h);
                            spriteCounter++;
                        }

                        if (drone.droneControlStatus.Contains("Recharg") || drone.droneControlStatus.Contains("Unload") || drone.drone_recharge_request.Contains("True"))
                        {
                            Image_drone = "IconEnergy";
                            bore_colour_drone = Color.Yellow;
                            var sprite_layer = new MySprite()
                            {
                                Type = SpriteType.TEXTURE,
                                Data = Image_drone,
                                Position = position,
                                //RotationOrScale = size_scale,
                                Size = sizer * 0.8f,
                                Color = bore_colour_drone.Alpha(alpha_val),
                                Alignment = TextAlignment.CENTER
                            };
                            sprites.Add(sprite_layer);
                            spriteCounter++;
                        }

                        var position_text = new Vector2(CentX * scale_factor_x, CentY * scale_factor_y) + _viewport.Center;
                        //background sprite
                        bore_colour_drone = Color.WhiteSmoke;
                        var sprite_name = new MySprite()
                        {
                            Type = SpriteType.TEXT,
                            Data = $"{drone.droneName}- ({drone.drone_charge_storage}%)",
                            Position = position,
                            RotationOrScale = 0.3f,
                            Size = sizer * 0.5f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER,
                            FontId = "White"
                        };
                        sprites.Add(sprite_name);
                        percent_list_drones = ((double)drone_total / (double)droneRegistry.Count) * 100;
                        spriteCounter++;
                        yield return false;
                    }
                }
                if (droneRegistry.Count == 0)
                {
                    if (sprite_total == gridBorePosition.Count)
                    {
                        frame_generator_finished = true;
                    }

                    else
                    {
                        frame_generator_finished = false;
                    }
                }
                if (droneRegistry.Count > 0)
                {
                    if (sprite_total == gridBorePosition.Count && drone_total == droneRegistry.Count)
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
                Echo("Frame shift");
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
        public void CyclNextCord()
        {
            currentGPSIndex = (currentGPSIndex + 1) % gridBorePosition.Count;
            //next_gps_crds = gridBorePosition[currentGPSIndex];
        }



        int CountTrueValues(List<bool> list)
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
        int CountIntegerValues(Dictionary<string, Drone> list, int val)
        {
            int truCnt = 0;

            foreach (var kvp in list)
            {
                Drone drone = kvp.Value;
                if (drone.droneGPSListPosition == val)
                {
                    truCnt++;
                }
            }
            return truCnt;
        }
        int CountStatusValues(List<string> list, string textval)
        {
            int trueCount = 0;

            foreach (string value in list)
            {
                if (value.Contains(textval))
                {
                    trueCount++;
                }
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
        void transmitToDrone(Drone drone)
        {
            IGC.SendBroadcastMessage(tx_chan, drone.droneTranmissionOutput, TransmissionDistance.TransmissionDistanceMax);
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

        void reset_drone_data()
        {
            droneRegistry.Clear();
        }

        void reset_drone_list()
        {
            droneName = new List<string>();
            droneDamageState = new List<string>();
            droneTunnelFinished = new List<string>();
            droneControlStatus = new List<string>();
            droneDocked = new List<string>();
            droneUndocked = new List<string>();
            droneAutopiloting = new List<string>();
            droneGPSListPosition = new List<int>();
            droneGPSCoordinates = new List<Vector3D>();
            droneBoreDepth = new List<string>();
            droneBoreDepthCurrent = new List<string>();
            drone_mine_depth_start_status = new List<string>();
            drone_location_x = new List<string>();
            drone_location_y = new List<string>();
            drone_location_z = new List<string>();
            drone_charge_storage = new List<string>();
            drone_gas_storage = new List<string>();
            drone_ore_storage = new List<string>();
            droneMining = new List<bool>();
            droneAssignedCoordinates = new List<bool>();
            droneControlSequence = new List<int>();
            droneRecallSequence = new List<int>();
            droneTranmissionOutput = new List<string>();
            droneReady = new List<bool>();
            droneMustWait = new List<bool>();
            dcs = new List<double>();
            dst = new List<bool>();
            droneTransmissionStatus = new List<bool>();
            droneRecallList = new List<bool>();
            droneResetFunction = new List<bool>();
            drone_assigns_count = new List<int>();
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
                Echo("No arguments provided, using defaults.");
                drone_tag = "SWRM_D";
                drone_length = 2.6;
                drone_clear_offset = 9.0; //drill clear mode distance offset
                secondary = ""; //vessel/rig name (optional)
                return;
            }

            string[] dronecontrolleronfigdata = input.Split(',');

            // Check if the split array is unexpectedly empty (though covered by the initial check)
            if (dronecontrolleronfigdata.Length == 0)
            {
                Echo("No arguments provided, using defaults.");
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
            if (dronecontrolleronfigdata.Length >= 2 && !string.IsNullOrWhiteSpace(dronecontrolleronfigdata[1]))
            {
                secondary = dronecontrolleronfigdata[1].ToString();
            }
            if (dronecontrolleronfigdata.Length >= 3)
            {
                if (!double.TryParse(dronecontrolleronfigdata[2].ToString().Trim(), out drone_length))
                {
                    drone_length = 2.6; // Set to default on fail
                }
            }
            else
            {
                drone_length = 2.6; // Default if argument is missing
            }
            if (dronecontrolleronfigdata.Length >= 4)
            {
                if (!double.TryParse(dronecontrolleronfigdata[3].ToString().Trim(), out drone_clear_offset))
                {
                    drone_clear_offset = 9.0; // Set to default on fail
                }
            }
            else
            {
                drone_clear_offset = 9.0; // Default if argument is missing
            }
        }
        public void drone_custom_data_check(string custominfo, int index)
        {
            Echo("Checking for drone config information..");
            String[] temp_id = custominfo.Split(':');
            Echo($"{temp_id.Length}");

            if (temp_id.Length > 0)
            {
                if (temp_id[0] != null)
                {
                    //temp_id_name = temp_id[0];
                    drone_tag = temp_id[0];
                    if (temp_id[0] == "" || temp_id[0] == null)
                    {
                        Echo($"Resorting to default scout tag {drone_tag}");
                    }
                }
            }
            else
            {
                //temp_id_name = drone_tag;
                Echo($"Resorting to default ID#.{drone_tag}");
            }
            if (temp_id.Length > 1)
            {
                if (temp_id[1] != null)
                {
                    //temp_id_name_2 = temp_id[1];
                    secondary = temp_id[1];
                    if (temp_id[1] == null)
                    {
                        //temp_id_name_2 = secondary;
                        Echo($"Resorting to default scout tag {secondary}");
                    }
                }
            }
            else
            {
                //temp_id_name_2 = secondary;
                Echo($"Resorting to default ID#.{secondary}");
            }
            if (temp_id.Length == 0)
            {
                //temp_id_name = drone_tag;
               // temp_id_name_2 = secondary;
                Echo($"Resorting to default config {drone_tag} {secondary}.");
            }


            if (antennaAll[index] != null)
            {
                antennaAll[index].CustomData = $"{drone_tag}:{secondary}:";
            }
            Echo($"Drone info:{drone_tag.Replace("[", "[[").Replace("]", "]]")}");
            antennaTagName = "[" + drone_tag + " " + comms + "]";
            lightsTagName = "[" + drone_tag + " " + comms + "]";
            dp_mn_tag = "[" + drone_tag + " " + MainS + " " + dspy + "]";
            dp_drn_tag = "[" + drone_tag + " " + DroneS + " " + dspy + "]";
            dp_lst_tag = "[" + drone_tag + " " + LstS + " " + dspy + "]";
            dp_vis_tag = "[" + drone_tag + " " + GrphS + " " + dspy + "]";
            interfaceTag = "[" + drone_tag + " " + IntfS + "]";
            secondary_tag = "[" + secondary + "]";
            rxChannelDrone = drone_tag + " " + replyC;
            rxChannelProspector = drone_tag + " " + prospC;
            tx_recall_channel = drone_tag + " " + commandRecall;
            txDronePingChannel = "[" + drone_tag + "]" + " " + pingMessage;
            txDroneSyncChannel = "[" + drone_tag + "]" + " " + syncC;
            syncMessage = secondary;
        }

        public void SetupSystem()
        {
            #region setup_system
            IMyGridTerminalSystem gts = GridTerminalSystem as IMyGridTerminalSystem;
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
            drone_location = new List<Vector3D>();
            droneName = new List<string>();
            droneDamageState = new List<string>();
            droneTunnelFinished = new List<string>();
            droneControlStatus = new List<string>();
            droneDocked = new List<string>();
            droneUndocked = new List<string>();
            droneAutopiloting = new List<string>();
            droneGPSCoordinates = new List<Vector3D>();
            droneControlSequence = new List<int>();
            droneGPSListPosition = new List<int>();
            droneAssignedCoordinates = new List<bool>();
            gridBorePosition = new List<Vector3D>();
            droneBoreDepth = new List<string>();
            droneBoreDepthCurrent = new List<string>();
            drone_mine_depth_start_status = new List<string>();
            droneMining = new List<bool>();
            drone_location_x = new List<string>();
            drone_location_y = new List<string>();
            drone_location_z = new List<string>();
            drone_charge_storage = new List<string>();
            drone_gas_storage = new List<string>();
            drone_ore_storage = new List<string>();
            droneTranmissionOutput = new List<string>();
            droneRecallSequence = new List<int>();
            droneReady = new List<bool>();
            droneMustWait = new List<bool>();
            droneRecallList = new List<bool>();
            drone_assigns_count = new List<int>();
            sprites = new List<MySprite>();
            remoteControlAll = new List<IMyRemoteControl>();
            remoteControlTag = new List<IMyRemoteControl>();
            droneAutodock = new List<string>();
            droneDockingReady = new List<string>();
            drone_cargo_full = new List<string>();
            drone_recharge_request = new List<string>();
            drone_auto_pilot_enabled = new List<string>();
            bores_regen = false;
            cl = new List<string>();
            cl2 = new List<string>();
            tla = new List<int>();
            rst = new List<int>();
            fct = new List<string>();
            dcs = new List<double>();
            dst = new List<bool>();
            droneTransmissionStatus = new List<bool>();
            miningCoordinatesNew = new StringBuilder();
            displayTextMain = new StringBuilder();
            displayTextList = new StringBuilder();
            droneInformation = new StringBuilder();
            droneResetFunction = new List<bool>();
            c = new StringBuilder();
            jxt = new StringBuilder();
            customDataString = new StringBuilder();
            _statusBuffer = new StringBuilder();
            droneRegistry = new Dictionary<string, Drone>();
            for (int i = 0; i < 12; i++)
            {
                cl.Add("");
                cl2.Add("");
                tla.Add(0);
                rst.Add(0);
                fct.Add("");
            }
            antennaAll = new List<IMyRadioAntenna>();
            antennaTag = new List<IMyRadioAntenna>();
            gts.GetBlocksOfType<IMyRadioAntenna>(antennaAll, b => b.CubeGrid == Me.CubeGrid);
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
            antennaAll.Clear();
            Me.CustomName = $"GMDC Programmable Block {secondary_tag} {antennaTagName}";
            gts.GetBlocksOfType<IMyRemoteControl>(remoteControlAll, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < remoteControlAll.Count; i++)
            {
                if (remoteControlAll[i].CustomName.Contains(antennaTagName) || remoteControlAll[i].CustomName.Contains(comms))
                {
                    remoteControlAll[i].CustomName = $"GMDC Remote Control {secondary_tag} {antennaTagName}";
                    remoteControlTag.Add(remoteControlAll[i]);
                }
            }
            remoteControlAll.Clear();

            lightsAll = new List<IMyLightingBlock>();
            lightsTag = new List<IMyLightingBlock>();
            gts.GetBlocksOfType<IMyLightingBlock>(lightsAll, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < lightsAll.Count; i++)
            {
                if (lightsAll[i].CustomName.Contains(lightsTagName) || lightsAll[i].CustomName.Contains(comms))
                {
                    lightsAll[i].CustomName = $"GMDC Indicator Light {secondary_tag} {lightsTagName}";
                    lightsTag.Add(lightsAll[i]);
                }
            }
            lightsAll.Clear();
            display_all = new List<IMyTerminalBlock>();
            display_tag_main = new List<IMyTerminalBlock>();
            display_tag_list = new List<IMyTerminalBlock>();
            display_tag_drone = new List<IMyTerminalBlock>();
            display_tag_vis = new List<IMyTerminalBlock>();
            gts.GetBlocksOfType<IMyTerminalBlock>(display_all, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < display_all.Count; i++)
            {
                if (display_all[i].CustomName.Contains(dp_mn_tag))
                {
                    display_tag_main.Add(display_all[i]);
                }
                if (display_all[i].CustomName.Contains(dp_drn_tag))
                {
                    display_tag_drone.Add(display_all[i]);
                }
                if (display_all[i].CustomName.Contains(dp_lst_tag))
                {
                    display_tag_list.Add(display_all[i]);
                }
                if (display_all[i].CustomName.Contains(dp_vis_tag))
                {
                    display_tag_vis.Add(display_all[i]);
                }
            }
            display_all.Clear();
            programblockAll = new List<IMyProgrammableBlock>();
            interfacePBTag = new List<IMyProgrammableBlock>();
            gts.GetBlocksOfType<IMyProgrammableBlock>(programblockAll, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < programblockAll.Count; i++)
            {
                if (programblockAll[i].CustomName.Contains(interfaceTag) || programblockAll[i].CustomName.Contains(IntfS))
                {
                    programblockAll[i].CustomName = $"GMDI Programmable Block {secondary_tag} {interfaceTag}";
                    interfacePBTag.Add(programblockAll[i]);
                }
            }
            programblockAll.Clear();

            droneMessagesBuffer = new List<MyIGCMessage>();
            prospectorMessagesBuffer = new List<MyIGCMessage>();

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

            if (display_tag_vis.Count > 0 && display_tag_vis[0] != null)
            {

                sV = ((IMyTextSurfaceProvider)display_tag_vis[0]).GetSurface(srfV);
                if (sV.ContentType != ContentType.SCRIPT)
                {
                    Echo("Correcting visualiser display");
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
            if (display_tag_vis.Count > 0 && display_tag_vis[0] != null)
            {
                Echo($"Display with tag '{dp_vis_tag.Replace("[", "[[").Replace("]", "]]")}' found");
                _viewport = new RectangleF((sV.TextureSize - sV.SurfaceSize) / 2f, sV.SurfaceSize);
                Visport_OK = true;
            }

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
                Echo($"Interface PB with tag: '{interfaceTag.Replace("[", "[[").Replace("]", "]]")}' not found.");
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
                if (sM.ContentType != ContentType.TEXT_AND_IMAGE)
                {
                    sM.ContentType = ContentType.TEXT_AND_IMAGE;
                    sM.FontSize = 0.65f;
                    sM.Font = "White";
                }
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
                if (sL.ContentType != ContentType.TEXT_AND_IMAGE)
                {
                    sL.ContentType = ContentType.TEXT_AND_IMAGE;
                    sL.FontSize = 0.66f;
                    sL.Font = "White";
                }
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
                if (sD.ContentType != ContentType.TEXT_AND_IMAGE)
                {
                    sD.ContentType = ContentType.TEXT_AND_IMAGE;
                    sD.FontSize = 0.296f;
                    sD.Font = "Monospace";

                }
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
            if (string.IsNullOrWhiteSpace(receivedDroneName))
            {
                Echo("Message Empty!");
                return;
            }

            Drone drone;

            if (!droneRegistry.TryGetValue(receivedDroneName, out drone))
            {
                drone = new Drone
                {
                    droneName = receivedDroneName
                };
                drone.droneMining = false;
                drone.droneAssignedCoordinates = false;
                drone.droneControlSequence = 0;
                drone.droneRecallSequence = 0;
                drone.droneTranmissionOutput = "";
                drone.droneReady = false;
                drone.droneMustWait = true;
                drone.dcs = 0.0;
                drone.dst = true;
                drone.droneRecallList = false;
                drone.droneResetFunction = false;
                drone.drone_assigns_count = 0;
                droneMessageConfirmed = true;
                Echo($"Processing Drone Info! {receivedDroneName.Replace("[","[[").Replace("]","]]")} {droneRegistry.Count}");
                droneRegistry.Add(receivedDroneName, drone);
            }
            Echo($"Processing Drone Info! {receivedDroneName.Replace("[", "[[").Replace("]", "]]")} {droneRegistry.Count}");
            drone.droneDamageState = receivedDroneDamageStatus;
            drone.droneTunnelFinished = receivedDroneTunnelFinished;
            drone.droneControlStatus = receivedDroneStatus;
            drone.droneDocked = receivedDroneDocked;
            drone.droneUndocked = receivedDroneUndocked;
            drone.droneAutopiloting = recived_drone_autopilot;
            drone.droneGPSListPosition = recieved_drone_list_position;
            drone.droneGPSCoordinates = remoteControlActual.GetPosition();
            drone.droneBoreDepth = rc_dn_drl_dpth;
            drone.droneBoreDepthCurrent = rc_dn_drl_crnt;
            drone.drone_mine_depth_start_status = rc_dn_drl_strt;
            drone.drone_location_x = rc_locx;
            drone.drone_location_y = rc_locy;
            drone.drone_location_z = rc_locz;
            drone.drone_charge_storage = rc_dn_chg;
            drone.drone_gas_storage = rc_dn_gas;
            drone.drone_ore_storage = rc_dn_str;
            drone.dcs = rc_d_cn;
            drone.droneTransmissionStatus = true;
            drone.drone_cargo_full = rc_dn_cargo_full;
            drone.drone_recharge_request = rc_dn_rchg_req;
            drone.drone_auto_pilot_enabled = rc_auto_pilot_enabled;
            drone.droneAutodock = recievedDroneAutdock;
            drone.droneDockingReady = recievedDroneDockingReady;
            if (drone.dcs <= bclm)
            {
                drone.dst = false;
            }
            if (drone.dcs > bclm)
            {
                drone.dst = true;
            }
            droneMessageConfirmed = true;
            //receivedDroneNameIndex = i;
        }

        void update_display()  // Extracted from drone_processing
        {
            displayTextMain.Clear().EnsureCapacity(512); // ~400-600 chars typical
            displayTextMain.AppendLine($"GMDC {ver} {secondary_tag} [{drone_tag}] Running {icon}")
                  .AppendLine($"------------------------------")
                  .AppendLine($" ")
                   .AppendLine($"Total drones detected: {droneRegistry.Count}")
                  .AppendLine(dronesLaunchedStatus
                      ? $"Drones active: {totalDronesActive} - Fault: {totalDronesDamaged} (Max: {maxActiveDronesCount} ({dronesInFlightFactor})) Hard limit: {dronesActiveHardLimit}"
                      : $"Drones active: {totalDronesActive} - Fault: {totalDronesDamaged}")
                  .AppendLine($"Docking: {t_drn_dckg} Docked: {t_drn_dck} - Unload: {t_drn_unload} Recharge: {t_drn_rechg} Idle: {t_drn_idle_docked}  ")
                  .AppendLine($"Undocking: {t_drn_udckg} Undocked: {t_drn_udck} - Idle: {t_drn_idle_undocked} Nav: {t_drn_nav} Mining: {t_drn_mine} Exit: {t_drn_exit}")
                  .AppendLine()
                  .AppendLine($"Surface distance: {safe_dstvl}m")
                  .AppendLine($"Drill depth: {drillLength}m ({drillLength + safe_dstvl}m)")
                  .AppendLine($"Req. ignore depth: {ignoreDepth}m (Drone length: {drone_length}m)")
                  .AppendLine($"Ignore depth: {safe_dstvl + drone_length - drone_clear_offset + ignoreDepth}m (Drill Start: {(drillLength + safe_dstvl) - (ignoreDepth + safe_dstvl + drone_length - drone_clear_offset)}m)")
                  .AppendLine()
                  .AppendLine($"Command: {commandAsk} Reset: {generalReset}")
                  .AppendLine($"Status: {screenStatus}")
                  .AppendLine()
                  .AppendLine("Target Coordinates:")
                  .AppendLine($"{miningGPSCoordinates}");
            if (prospectAlignTargetValid || customDataAlignTargetValid) displayTextMain.AppendLine("Align Coordinates:").AppendLine(alignGPSCoordinates.ToString());

            if (display_tag_main.Count > 0 && sM != null) sM.WriteText(displayTextMain);
        }



        // For reference types (classes)

        //program end
        public void StoreRawInput(string inputString, IMyTerminalBlock block, string INI_SECTION = "GMDCJobData", string INI_KEY = "Jobinfo")
        {
            var iniBuilder = new MyIni();
            // 1. Correct MyIni.Set() usage: (Section, Key, Value)
            iniBuilder.Set(INI_SECTION, INI_KEY, inputString);

            // Save to the Programmable Block's CustomData
            block.CustomData = iniBuilder.ToString();
            Echo($"Raw input stored successfully in [{INI_SECTION}] {INI_KEY}.");
        }


        double ParseDouble(string[] data, int index, double defaultValue)
        {
            double result;
            if (index < data.Length && double.TryParse(data[index], out result)) return result;
            return defaultValue;
        }

        int ParseInt(string[] data, int index, int defaultValue)
        {
            int result;
            if (index < data.Length && int.TryParse(data[index], out result)) return result;
            return defaultValue;
        }

        public void ClearCustomDataVariables()
        {
            //customData1 = "";
            //customData2 = "";
            //customData3 = "";
           // customData4 = "";
            customData5 = "";
           // customData6 = "";
            customData7 = "";
            customData8 = "";
            customData9 = "";
          //  customData10 = "";
          //  customData11 = "";
          //  customData12 = "";
          //  customData13 = "";
         //   customData14 = "";
         //  customData15 = "";
            //customData16 = "";
            //customData17 = "";
            //customData18 = "";
            //customData19 = "";
            //customData20 = "";
            //customData21 = "";
            //customData22 = "";
        }


    }

}

