using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
using VRage.Game.GUI.TextPanel;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        // R e a d m e
        // -----------
        // GMDC Drone controller
        // 


        #region mdk preserve
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }
        //default information
        string drone_tag = "SWRM_D"; //Mining drone group tag
        double drone_length = 2.6;
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
        string ver = "V0.389B";
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
        string pingMessage = "ping";
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
        string commandAsk;
        string customData1;
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
        string customData16;
        string customData17;
        string customData18;
        string customData19;
        string customData20;
        string customData21;
        string remoteControlCustomData1 = "";
        string remoteControlCustomData2 = "";
        string remoteControlCustomData3 = "";
        string remoteControlCustomData4 = "";
        string remoteControlCustomData5 = "";
        string remoteControlCustomData6 = "";
        string remoteControlCustomData7 = "";
        string remoteControlCustomData8 = "";
        string remoteControlCustomData9 = "";
        string remoteControlCustomData10 = "";
        string remoteControlCustomData11 = "";
        string remoteControlCustomData12 = "";

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
        int totalDronesMining = 0;
        int t_drn_dckg = 0;
        int t_drn_dck = 0;
        int t_drn_udckg = 0;
        int t_drn_udck = 0;
        int t_drn_rechg = 0;
        int t_drn_unload = 0;
        int t_drn_idle = 0;
        int t_drn_exit = 0;
        int t_drn_mine = 0;
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
        Vector3D next_gps_crds;
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
        List<IMyRemoteControl> remoteControlAll;
        List<IMyRemoteControl> remoteControlTag;
        List<IMyRadioAntenna> antennaAll;
        List<IMyRadioAntenna> antennaTag;
        List<IMyLightingBlock> lightsAll;
        List<IMyLightingBlock> lightsTag;
        List<IMyTerminalBlock> display_all;
        List<IMyTerminalBlock> display_tag_main;
        List<IMyTerminalBlock> display_tag_list;
        List<IMyTerminalBlock> display_tag_drone;
        List<IMyTerminalBlock> display_tag_vis;
        List<IMyProgrammableBlock> programblockAll;
        List<IMyProgrammableBlock> interfacePBTag;
        IMyTextSurface sD;
        IMyTextSurface sM;
        IMyTextSurface sL;
        IMyTextSurface sV;
        RectangleF _viewport;
        StringBuilder sb;
        int totalDronesDamaged = 0;
        int totalDronesUnknown = 0;
        int t_dn_ok = 0;
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
        int debugcount = 0;
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
        string temp_id_name;
        string temp_id_name_2;
        string secondary_tag = "";
        double game_tick_length = 16.666;

        IMyBroadcastListener listenDrones;
        IMyBroadcastListener listenProspector;
        List<MyIGCMessage> droneMessagesBuffer;
        List<MyIGCMessage> prospectorMessagesBuffer;
        bool prospectorMessageReceived = false;
        bool droneMessageConfirmed = false;
        int receivedDroneNameIndex = -1;
        bool droneMessageReceived = false;
        bool Visport_OK = false;
        List<MySprite> sprites;
        int spriteCounter = 0;
        bool spriteInsert = false;
        StringBuilder customDataString;

        private double totalRuntimeMs = 0.0;
        private int runCount = 0;
        private double averageRuntimeMs = 0.0;
        #endregion
        public void Save()
        {
            if (setupComplete)
            {
                sb = new StringBuilder();
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
                    Storage = sb.ToString();
                    sb.Clear();
                }
            }
        }


        public void Main(string argument, UpdateType updateSource)
        {
            Echo("Running Modular Main - v1");
            int startInstructions = Runtime.CurrentInstructionCount;
            UpdateRuntimeMetrics(updateSource);
            InitializeSystem();
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
            totalRuntimeMs += _Runtime;
            runCount++;
            if (runCount == 10)
            {
                averageRuntimeMs = totalRuntimeMs / runCount;
                runCount = 0;
                totalRuntimeMs = 0;
            }
            Echo($"UpdateRuntimeMetrics: {Runtime.CurrentInstructionCount - startInstructions}");
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
            Echo($"GMDC {ver} Running {icon}");
            Echo($"Channel: {drone_tag}");
            Echo($"InitializeSystem: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ProcessInputs(string argument)
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            ProcessInterface();
            HandleCommands(argument);
            Echo($"ProcessInputs: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ManageCommunications()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            listenDrones = IGC.RegisterBroadcastListener(rxChannelDrone);
            listenProspector = IGC.RegisterBroadcastListener(rxChannelProspector);
            ProcessMessages();
            Echo($"ManageCommunications: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateMiningGrid()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            InitializeMiningGrid();
            ValidateCustomData();
            PingDrones();
            GetRemoteControlData();
            //Echo($"Pre-Prospect: valid= RC: {prospectTargetValid}  , ALN: {prospectAlignTargetValid}, coords= PB: {miningGPSCoordinates} RC: {targetGPSCoordinates}");
            if (prospectAlignTargetValid)
            {
                //Echo($"Pre-Prospect:  Align coords={alignGPSCoordinates}");
            }
            if (prospectorMessageReceived)
            {
                Storage = null;
                prospectAlignTargetValid = false;
                GetRemoteControlData();
                //Echo($"Post-Prospect: valid={prospectTargetValid}, {prospectAlignTargetValid}, coords={targetGPSCoordinates}");
                if (prospectAlignTargetValid)
                {
                    Echo($"Post-Prospect: Main PB: {miningCoordsValid} Align coords=:{alignGPSCoordinates}");
                }
                if (prospectTargetValid)
                {
                    Echo($"Formatting CustomData with: {targetGPSCoordinates.X}, {targetGPSCoordinates.Y}, {targetGPSCoordinates.Z}");
                    miningCoordinatesNew.Clear().AppendFormat("GPS:PDT:{0:0.##}:{1:0.##}:{2:0.##}:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:",
                        targetGPSCoordinates.X, targetGPSCoordinates.Y, targetGPSCoordinates.Z);
                    if (prospectAlignTargetValid)
                    {
                        miningCoordinatesNew.Append($"GPS:TGT:{alignGPSCoordinates.X}:{alignGPSCoordinates.Y}:{alignGPSCoordinates.Z}:");
                    }
                    Me.CustomData = miningCoordinatesNew.ToString();
                }
                prospectorMessageReceived = false;
                gridCreated = false;
            }
            GetCustomDataJobCommand();
            ProcessJobGrid();
            UpdateActiveDroneLimits();
            Echo($"UpdateMiningGrid: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void HandleDroneOperations()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (droneName.Count > 0 && gridCreated && timeDelayed)
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
            Echo($"HandleDroneOperations: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void RenderDisplays()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            DroneRenderCall();
            ListRenderCall();
            SpriteRenderCall();
            Echo($"RenderDisplays: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateStatus()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            TimeCounterReset();
            LocalStatusUpdate(Runtime.LastRunTimeMs);
            Echo($"UpdateStatus: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void LocalStatusUpdate(double _Runtime)
        {
            Echo($"Load: {Math.Round((_Runtime / game_tick_length) * (double)100.0, 3)}% ({Math.Round(_Runtime, 3)}ms) S#:{spriteCounter} {spriteInsert}");
            Echo($"Drones #: {droneName.Count}");
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
                        Echo($"Frame reset - spritecount {spriteCounter}");
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
            if(lightIndicatorActual == null || lightsTag[0] == null)
            {
                Echo($"Indicator light missing {lightsTagName} - early exit");
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

            if (totalDronesMining > 0 && gridBoresCompleted < totalMiningRuns && canTransmit && canRun || faultLightOutput)
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
                droneResetStatusCount = CountIntegerValues(droneGPSListPosition, -1);
                droneDockedStatusCount = CountStatusValues(droneControlStatus, "Docked");
            }
            if (droneName.Count > 0)
            {
                if (droneResetStatusCount == droneName.Count && droneDockedStatusCount == droneName.Count && canReset)
                {
                    readyFlag = true;
                }
            }
        }

        private void ProcessDroneState()
        {
            #region drone_state_machine_management
            if (receivedDroneNameIndex != -1 && droneMessageConfirmed)
            {

                int i = receivedDroneNameIndex;

                if (canInit || canReset || droneResetFunction[i] || canLoading)
                {
                    generalReset = true;
                }
                else generalReset = false;
                di = i;
                faultCounter = CountTrueValues(dst);
                if (faultCounter < droneName.Count)
                {
                    faultLightOutput = true;
                }
                else faultLightOutput = false;

                //recall sequence reset - global
                if (!droneRecallList[i] || canReset || canInit || canLoading)
                {
                    droneRecallSequence[i] = 0;
                }
                displayTextMain.Clear();

                if (droneGPSListPosition[i] > -1 && !droneAssignedCoordinates[i])
                {
                    droneGPSListPosition[i] = -1;
                }
                //if undocked request local recall sequence flag to ON
                if (droneGPSListPosition[i] == -1 && !droneAssignedCoordinates[i] && droneUndocked[i] == "True" && droneDocked[i] == "False" && !droneRecallList[i] && !mustUndockCommand || droneGPSListPosition[i] == -1 && !droneAssignedCoordinates[i] && droneUndocked[i] == "False" && droneDocked[i] == "False" && !droneRecallList[i] && !mustUndockCommand)
                {
                    droneRecallList[i] = true;
                }
                if (droneRecallList[i])
                {
                    droneTXRecallChannel = droneName[i] + " " + commandRecall;
                    IGC.SendBroadcastMessage(droneTXRecallChannel, commandRecall, TransmissionDistance.TransmissionDistanceMax);
                }
                if (!droneRecallList[i])
                {
                    droneTXRecallChannel = droneName[i] + " " + commandRecall;
                    IGC.SendBroadcastMessage(droneTXRecallChannel, commandOperate, TransmissionDistance.TransmissionDistanceMax);
                }


                if (droneControlStatus[i].Contains("Docked") && droneGPSListPosition[i] == -1 && droneMining[i] && droneControlSequence[i] == 0)
                {
                    droneMining[i] = false;
                }
                if (totalDronesMining >= boresRemaining && !droneMining[i] && gridBoresCompleted <= totalMiningRuns || boresRemaining == 0 && droneMining[i] == false)
                {
                    if (!dronesLaunchedStatus || dronesUndocking)
                    {
                        droneMustWait[i] = true;
                    }
                    if (dronesLaunchedStatus && totalDronesMining > maxActiveDronesCount || dronesUndocking)
                    {
                        droneMustWait[i] = true;
                    }
                    if (dronesLaunchedStatus && totalDronesMining <= maxActiveDronesCount)
                    {
                        droneMustWait[i] = false;
                    }
                }
                else if (totalDronesMining < boresRemaining && gridBoresCompleted < totalMiningRuns || droneMining[i] && totalDronesMining <= boresRemaining)
                {
                    if (!dronesLaunchedStatus)
                    {
                        droneMustWait[i] = false;
                    }
                    if (dronesLaunchedStatus && totalDronesMining < maxActiveDronesCount)
                    {
                        droneMustWait[i] = false;
                    }

                    if (dronesLaunchedStatus && totalDronesMining > maxActiveDronesCount || dronesUndocking)
                    {
                        droneMustWait[i] = true;
                    }
                }
                if (droneGPSListPosition[i] == -1 && totalDronesMining >= boresRemaining || dronesUndocking)
                {
                    if (!dronesLaunchedStatus)
                    {
                        droneMustWait[i] = true;
                    }
                    if (dronesLaunchedStatus && totalDronesMining >= maxActiveDronesCount || dronesUndocking)
                    {
                        droneMustWait[i] = true;
                    }
                    if (dronesLaunchedStatus && totalDronesMining < maxActiveDronesCount || dronesUndocking)
                    {
                        droneMustWait[i] = true;
                    }
                }
                if (droneGPSListPosition[i] > -1 && droneGPSListPosition[i] < gridBorePosition.Count)
                {
                    if (gridBoreOccupied[droneGPSListPosition[i]] && !droneMining[i])
                    {
                        if (!dronesLaunchedStatus || dronesUndocking)
                        {
                            droneMustWait[i] = true;
                        }
                        if (dronesLaunchedStatus && totalDronesMining >= maxActiveDronesCount || dronesUndocking)
                        {
                            droneMustWait[i] = true;
                        }
                        if (dronesLaunchedStatus && totalDronesMining < maxActiveDronesCount || dronesUndocking)
                        {
                            droneMustWait[i] = true;
                        }
                    }
                    else if (gridBoresCompleted < totalMiningRuns && !gridBoreOccupied[droneGPSListPosition[i]] && !gridBoreFinished[droneGPSListPosition[i]] && !droneMining[i])
                    {
                        if (!dronesLaunchedStatus)
                        {
                            droneMustWait[i] = false;
                        }
                        if (dronesLaunchedStatus && totalDronesMining < maxActiveDronesCount)
                        {
                            droneMustWait[i] = false;
                        }
                        if (dronesLaunchedStatus && totalDronesMining >= maxActiveDronesCount || dronesUndocking)
                        {
                            droneMustWait[i] = true;
                        }
                    }
                    if (!gridBoreFinished[droneGPSListPosition[i]])
                    {
                        int queued_count = CountIntegerValues(droneGPSListPosition, droneGPSListPosition[i]);
                        if (gridBoreOccupied[droneGPSListPosition[i]] && queued_count == 0)
                        {
                            gridBoreOccupied[droneGPSListPosition[i]] = false;
                        }

                    }
                }

                updateDisplay(i);

                gpsGridPositionValue = droneGPSListPosition[i];
                if (droneControlStatus[i] == "Docked Idle")
                {
                    droneReady[i] = true;
                }
                if (droneControlStatus[i].Contains("Recharging") || droneControlStatus[i].Contains("Unloading"))
                {
                    droneReady[i] = false;
                }
                if (droneReady[i] && droneTunnelFinished[i] == "False" && droneDocked[i] == "True" && canRun && !droneAssignedCoordinates[i] && droneControlSequence[i] == 0 && !droneMustWait[i] && !droneMining[i] && !disableRunArgument)
                {
                    if (gridBoresCompleted < totalMiningRuns && miningGridValid != false && !droneAssignedCoordinates[i] && droneMustWait[i] == false)
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
                            droneGPSCoordinates[i] = gridBorePosition[gpsGridPositionValue];
                            droneGPSListPosition[i] = gpsGridPositionValue;
                        }
                        else
                        {
                            gpsGridPositionValue = droneGPSListPosition[i];
                            droneGPSCoordinates[i] = gridBorePosition[gpsGridPositionValue];
                        }
                        if (!miningGridValid)
                        {
                            totalMiningRuns = 1;
                            droneGPSCoordinates[i] = miningGPSCoordinates;
                            gpsGridPositionValue = 0;
                            currentGPSIndex = 0;
                        }
                        //suspect code here
                        Echo($"Drone coords: {i}");
                        droneAssignedCoordinates[i] = true;
                        Echo($"Drone coords assigned: {i} {droneAssignedCoordinates[i]}");
                    }
                    else if (!miningGridValid)
                    {
                        totalMiningRuns = 1;
                        droneGPSCoordinates[i] = miningGPSCoordinates;
                        droneAssignedCoordinates[i] = true;
                        gpsGridPositionValue = 0;
                        currentGPSIndex = 0;
                    }
                    if (droneGPSListPosition[i] > -1)
                    {
                        if (gridBoreOccupied[droneGPSListPosition[i]] && !droneMining[i])
                        {
                            droneMustWait[i] = true;
                        }
                        else if (totalDronesMining < boresRemaining && gridBoresCompleted < totalMiningRuns || !gridBoreOccupied[droneGPSListPosition[i]] && !gridBoreFinished[droneGPSListPosition[i]] && !droneMining[i])
                        {
                            droneMustWait[i] = false;
                        }
                        if (gridBoresCompleted != totalMiningRuns && !droneMustWait[i])
                        {
                            droneControlSequence[i] = 1;
                            droneMining[i] = true;
                            gridBoreOccupied[droneGPSListPosition[i]] = true;
                        }
                        else
                        {
                            droneControlSequence[i] = 0;
                            droneMining[i] = false;
                        }
                        if (gridBoreFinished[droneGPSListPosition[i]])
                        {
                            //suspect coordinates here 2
                            Echo($"Drone position finished {i}");
                            droneControlSequence[i] = 0;
                            droneMining[i] = false;
                            droneAssignedCoordinates[i] = false;
                            droneGPSListPosition[i] = -1;
                        }
                    }
                }
                tx_chan = droneName[i];
                cd1 = gpsGridPositionValue.ToString();
                cm = "0";
                xp = Math.Round(droneGPSCoordinates[i].X, 2).ToString();
                yp = Math.Round(droneGPSCoordinates[i].Y, 2).ToString();
                zp = Math.Round(droneGPSCoordinates[i].Z, 2).ToString();
                cd5 = customData5;
                cd6 = (drillLength + safe_dstvl).ToString();
                igd = (ignoreDepth + safe_dstvl + drone_length).ToString();
                if (prospectAlignTargetValid)
                {
                    xp2 = Math.Round(((droneGPSCoordinates[i].X - miningGPSCoordinates.X) + alignGPSCoordinates.X), 2).ToString();
                    yp2 = Math.Round(((droneGPSCoordinates[i].Y - miningGPSCoordinates.Y) + alignGPSCoordinates.Y), 2).ToString();
                    zp2 = Math.Round(((droneGPSCoordinates[i].Z - miningGPSCoordinates.Z) + alignGPSCoordinates.Z), 2).ToString();
                }
                else
                {
                    xp2 = "";
                    yp2 = "";
                    zp2 = "";
                }
                if (droneControlSequence[i] == 1 && droneAssignedCoordinates[i] && !droneMustWait[i] && !disableRunArgument || droneControlSequence[i] == 2 && droneControlStatus[i] == "Docked Idle" && droneDocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 2;
                    droneMining[i] = true;
                    gridBoreOccupied[droneGPSListPosition[i]] = true;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "7";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 2 && droneControlStatus[i] == "Undocked" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument || droneControlSequence[i] == 2 && droneControlStatus[i] == "Docking" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 3;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (droneControlSequence[i] == 2 && droneControlStatus[i] == "Undocking" && droneDocked[i] == "False" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument && dcs[i] <= bclu)
                {
                    droneControlSequence[i] = 13;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 8 && droneControlStatus[i].Contains("RTB Ready") && droneDocked[i] == "False" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 13;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 13 && droneControlStatus[i] == "Idle" && droneDocked[i] == "False" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument || droneControlSequence[i] == 5 && droneControlStatus[i] == "Docking" && droneDocked[i] == "False" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument && dcs[i] <= bclu)
                {
                    droneControlSequence[i] = 8;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "6";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 3 && droneControlStatus[i] == "Idle" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 4;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "4";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 4 && droneControlStatus[i] == "Nav End" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 5;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 4 && droneControlStatus[i] == "Docked Idle" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (droneControlSequence[i] == 5 && droneControlStatus[i] == "Idle" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 6;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "2";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (droneControlSequence[i] == 6 && droneControlStatus[i] == "Nav End" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 7;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 7 && droneControlStatus[i] == "Idle" && droneUndocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 8;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "5";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] >= 8 && droneControlStatus[i].Contains("Docked") && droneMining[i] || droneControlSequence[i] == 4 && droneControlStatus[i].Contains("Docked") && droneMining[i])
                {
                    gridBoreOccupied[droneGPSListPosition[i]] = false;
                }
                if (droneControlSequence[i] >= 8 && droneControlStatus[i].Contains("Dock") && droneMining[i] && droneTunnelFinished[i] == "True")
                {
                    gridBoreFinished[droneGPSListPosition[i]] = true;
                }
                if (droneControlSequence[i] == 8 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && droneAssignedCoordinates[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 8 && !droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && droneAssignedCoordinates[i] && !disableRunArgument || droneControlSequence[i] == 8 && !droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && droneAssignedCoordinates[i] && !disableRunArgument || droneControlSequence[i] >= 1 && droneControlSequence[i] <= 4 && !droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && droneAssignedCoordinates[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 0;
                    droneAssignedCoordinates[i] = false;
                    droneMining[i] = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 8 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && droneAssignedCoordinates[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 9;
                    gridBoreFinished[droneGPSListPosition[i]] = true;

                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 9 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && canRun && droneAssignedCoordinates[i] && !disableRunArgument || droneControlSequence[i] == 9 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && droneAssignedCoordinates[i] && droneAssignedCoordinates[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 10;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 10 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && generalReset && droneAssignedCoordinates[i] && !disableRunArgument || droneControlSequence[i] == 10 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && droneAssignedCoordinates[i] && !disableRunArgument || droneControlSequence[i] == 0 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && droneAssignedCoordinates[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 11;
                    droneTunnelFinished[i] = "False";
                    gridBoreFinished[droneGPSListPosition[i]] = true;
                    totalMiningSequencesComplete++;
                    gpsGridPositionValue = -1;
                    droneResetFunction[i] = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "8";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 11 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && droneAssignedCoordinates[i] && totalMiningSequencesComplete <= totalMiningRuns && miningGridValid && !disableRunArgument)
                {
                    droneControlSequence[i] = 0;
                    droneAssignedCoordinates[i] = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlSequence[i] == 11 && droneControlStatus[i].Contains("Docked") && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && currentGPSIndex < totalMiningRuns && droneAssignedCoordinates[i] && totalMiningSequencesComplete > totalMiningRuns && !disableRunArgument || droneControlSequence[i] == 11 && droneReady[i] && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && droneAssignedCoordinates[i] && miningGridValid == false && totalMiningSequencesComplete >= totalMiningRuns && !disableRunArgument)
                {
                    droneControlSequence[i] = 12;
                    droneAssignedCoordinates[i] = false;
                    gpsGridPositionValue = -1;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }

                    displayTextMain.Append('\n');
                    displayTextMain.Append("Mining seq. complete");
                }
                if (droneControlStatus[i].Contains("Docked") && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && generalReset || droneControlStatus[i].Contains("Docked") && droneDocked[i] == "True" && droneTunnelFinished[i] == "True" && generalReset && !disableRunArgument)
                {
                    droneControlSequence[i] = 0;
                    totalMiningSequencesComplete = 0;
                    droneTunnelFinished[i] = "False";
                    droneAssignedCoordinates[i] = false;
                    droneMining[i] = false;
                    currentGPSIndex = 0;
                    gpsGridPositionValue = -1;
                    droneResetFunction[i] = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "8";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (droneControlStatus[i].Contains("Docked") && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && generalReset && droneControlSequence[i] == 0 && !disableRunArgument || droneControlStatus[i].Contains("Docked") && droneDocked[i] == "True" && droneTunnelFinished[i] == "False" && generalReset && !disableRunArgument || droneControlSequence[i] == 6 && droneControlStatus[i] == "Docked Idle" && droneDocked[i] == "True" && droneAssignedCoordinates[i] && droneMining[i] && !disableRunArgument)
                {
                    droneControlSequence[i] = 0;
                    totalMiningSequencesComplete = 0;
                    droneTunnelFinished[i] = "False";
                    droneAssignedCoordinates[i] = false;
                    droneMining[i] = false;
                    currentGPSIndex = 0;
                    gpsGridPositionValue = -1;
                    droneResetFunction[i] = false;
                    cd1 = gpsGridPositionValue.ToString();
                    cm = "0";
                    droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (mustRecall_Command && !droneRecallList[i] && !mustUndockCommand)
                {
                    droneRecallList[i] = true;
                }
                if (droneRecallList[i])
                {
                    if (droneRecallSequence[i] == 0 && droneControlStatus[i] == "Idle" || droneRecallSequence[i] == 0 && droneControlStatus[i] == "Undocked" || droneRecallSequence[i] == 0 && droneControlStatus[i] == "Nav" || droneRecallSequence[i] == 0 && droneControlStatus[i] == "Undocking" || droneRecallSequence[i] == 0 && droneControlStatus[i] == "Docking" || droneRecallSequence[i] == 0 && droneControlStatus[i] == "Initiating mining")
                    {
                        droneRecallSequence[i] = 1;
                    }

                    if (droneRecallSequence[i] == 0 && droneControlStatus[i] == "Nav End")
                    {
                        droneRecallSequence[i] = 3;
                    }
                    if (droneRecallSequence[i] == 1)
                    {
                        droneRecallSequence[i] = 2;
                        droneControlSequence[i] = 0;
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();
                    }
                    if (droneRecallSequence[i] == 2 && droneControlStatus[i] == "Idle")
                    {
                        droneRecallSequence[i] = 3;
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "1";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (droneRecallSequence[i] == 3 && droneControlStatus[i] == "Nav End")
                    {
                        droneRecallSequence[i] = 4;
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (droneRecallSequence[i] == 3 && droneControlStatus[i] == "Nav" && droneGPSListPosition[i] == -1 || droneRecallSequence[i] == 3 && droneControlStatus[i] == "Idle" && droneGPSListPosition[i] >= -1)
                    {
                        droneRecallSequence[i] = 4;
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();

                    }

                    if (droneRecallSequence[i] == 4 && droneControlStatus[i] == "Idle")
                    {
                        droneRecallSequence[i] = 5;
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "6";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (droneRecallSequence[i] == 4 && droneControlStatus[i] == "Idle")
                    {
                        droneRecallSequence[i] = 5;
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "6";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (droneRecallSequence[i] == 5 && droneControlStatus[i].Contains("Docked") || droneRecallSequence[i] == 0 && droneControlStatus[i].Contains("Docked"))
                    {
                        droneRecallSequence[i] = 0;
                        droneAssignedCoordinates[i] = false;
                        droneRecallList[i] = false;
                        droneMining[i] = false;
                        gpsGridPositionValue = -1;
                        droneResetFunction[i] = true;
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmitToDrone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (mustUndockCommand)
                {
                    if (droneControlStatus[i] == "Docked Idle")
                    {
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "7";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();
                        if (canTransmit && droneTransmissionStatus[i])
                        {
                            transmitToDrone();
                            droneTransmissionStatus[i] = false;
                        }
                    }
                }

                if (mustFreeze_Command)
                {
                    if (droneControlStatus[i] == "Undocked" || droneControlStatus[i] == "Idle")
                    {
                        gpsGridPositionValue = droneGPSListPosition[i];
                        cd1 = gpsGridPositionValue.ToString();
                        cm = "0";
                        droneCommandBuilder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneTranmissionOutput[i] = c.ToString();
                        if (canTransmit && droneTransmissionStatus[i])
                        {
                            transmitToDrone();
                            droneTransmissionStatus[i] = false;
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
            if (droneName.Count > 0)
            {
                maxActiveDronesCount = droneName.Count - dronesInFlightFactor;
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
            if(pbInterfaceActual == null || interfacePBTag[0] == null)
            {
                Echo($"Interface PB not found {interfaceTag}");
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
                    Echo($"Remote control {antennaTagName} not present - early exit");
                    return;
                }
                    Vector3D gravity = remoteControlActual.GetNaturalGravity();
                
                    if (prospectAlignTargetValid)
                {
                    planeNrml = ((miningGPSCoordinates - alignGPSCoordinates));
                }
                if (!prospectAlignTargetValid)
                {
                    planeNrml = gravity;
                }

                planeNrml.Normalize();
                Vector3D perpendicularVector = Vector3D.CalculatePerpendicularVector(planeNrml);
                perpendicularVector.Normalize();
                Vector3D centerPoint = miningGPSCoordinates;
                //load from storage if present (test required)
                if (Storage != null && Storage != "" && !gridCreated && bores_regen && !gridInitialisationComplete)
                {
                    //added from init
                    currentGPSIndex = 0;
                    realGPSIndex = currentGPSIndex;
                    GetStoredData();
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
                            debugcount++;
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
                if (droneMessagesBuffer.Count < droneName.Count)
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
                if (droneMessagesBuffer.Count > droneName.Count)
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
                        remoteControlActual.CustomData = prospectorDataInput;
                    }
                    else
                    {
                        Echo($"Remote control {remoteControlTag} not present");
                        return;
                    }
                    prospectorMessagesBuffer.RemoveAt(0);
                    gridCreated = false;
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
                if (droneName.Count == 0 && !dronesPinged || droneName.Count > 0 && !dronesPinged)
                {
                    IGC.SendBroadcastMessage(txDronePingChannel, pingMessage, TransmissionDistance.TransmissionDistanceMax);
                    dronesPinged = true;
                    dronePingTimerCount = 0;
                }
            }
            #endregion
        }

        private void ValidateCustomData()
        {
            #region check_custom_data_validation
            if (!string.IsNullOrEmpty(Me.CustomData))
            {
                mainCustomDataValid = true;
            }
            else
            {
                Echo($"Job custom data invalid - initialising job data");
                mainCustomDataValid = false;
                miningCoordinatesNew.Clear();
                miningCoordinatesNew.Append($"GPS:---:0:0:0:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:");
                Me.CustomData = miningCoordinatesNew.ToString();
            }
            if (mainCustomDataValid)
            {
                GetCustomDataJobCommand();
            }
            #endregion
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
                    Echo($"Interface programmable block not present {interfaceTag}");
                    return;
                }
                canInterfaceCommand = true;
                interfaceArgument = pbInterfaceActual.CustomData;
                Echo($"Interface PB: {interfaceTag}");
                Echo($"Display command: {interfaceArgument}");
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
            if (display_tag_drone.Count == 0 || droneName.Count == 0 || display_tag_drone[0] == null) return;

            if (renew_header)
            {
                droneInformation.Clear().Append($"Mining Drone Status - GMDC {ver}\n");
                renew_header = false;
            }

            int dronesPerDisplay = drones_per_screen * display_tag_drone.Count;
            if (dronesPerDisplay < droneName.Count)
            {
                Echo($"Insufficient displays '{dp_drn_tag}': {dronesPerDisplay} < {droneName.Count}");
                return;
            }

            for (int i = 0; i < droneName.Count; i += 2)
            {
                bool hasPair = i + 1 < droneName.Count;
                scrnbldr(i, hasPair ? i + 1 : i, hasPair);

                int displayIndex = i / drones_per_screen;
                if (displayIndex < display_tag_drone.Count && display_tag_drone[displayIndex] != null &&
                    (i % drones_per_screen == drones_per_screen - 2 || i >= droneName.Count - 2))
                {
                    sD = ((IMyTextSurfaceProvider)display_tag_drone[displayIndex]).GetSurface(srfD);
                    sD.WriteText(droneInformation);
                    renew_header = true;
                }
            }
            Echo($"drone_render_call: {Runtime.CurrentInstructionCount - startInstructions}");
        }
        private struct DroneStats
        {
            public int Docking, Docked, Undocking, Undocked, Damage, Unknown, Ok, Exit, Idle, Recharge, Unload, Mining, RTBA, RTBB;
        }

        private void UpdateDroneCounts()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            gridBoresCompleted = CountTrueValues(gridBoreFinished);
            boresRemaining = totalMiningRuns - gridBoresCompleted;
            totalDronesMining = CountTrueValues(droneMining);
            total_drones_undocking = CountIntegerValues(droneControlSequence, 2);

            DroneStats stats = new DroneStats();
            for (int i = 0; i < droneName.Count; i++)
            {
                string status = droneControlStatus[i];
                string damage = droneDamageState[i];
                stats.Docking += status.Contains("Docking") ? 1 : 0;
                stats.Docked += status.Contains("Docked") ? 1 : 0;
                stats.Undocking += status.Contains("Undocking") ? 1 : 0;
                stats.Undocked += status.Contains("Undocked") ? 1 : 0;
                stats.Exit += status.Contains("Exit") ? 1 : 0;
                stats.Idle += status.Contains("Idle") ? 1 : 0;
                stats.Recharge += status.Contains("Recharg") ? 1 : 0;
                stats.Unload += status.Contains("Unload") ? 1 : 0;
                stats.Mining += status.Contains("Min") ? 1 : 0;
                stats.RTBA += status.Contains("RTB: Request") ? 1 : 0;
                stats.RTBB += status.Contains("RTB: Ready") ? 1 : 0;
                stats.Damage += damage == "DMG" ? 1 : 0;
                stats.Unknown += damage == "UNK" ? 1 : 0;
                stats.Ok += damage == "OK" ? 1 : 0;
            }
            t_drn_dckg = stats.Docking; t_drn_dck = stats.Docked; t_drn_udckg = stats.Undocking;
            t_drn_udck = stats.Undocked; t_drn_exit = stats.Exit; t_drn_idle = stats.Idle;
            t_drn_rechg = stats.Recharge; t_drn_unload = stats.Unload; t_drn_mine = stats.Mining;
            totalDronesDamaged = stats.Damage; totalDronesUnknown = stats.Unknown; t_dn_ok = stats.Ok;

            Echo($"UpdateDroneCounts: {Runtime.CurrentInstructionCount - startInstructions}");
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
            Echo($"DroneUndockCheck: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        public void updateDisplay(int i)
        {
            displayTextMain.Append($"Drone Controller Status - GMDC {ver} - [{drone_tag}] {icon}");
            displayTextMain.Append('\n');
            if (droneControlSequence[i] == 12)
            {
                gpsGridPositionValue = -1;
                droneMining[i] = false;
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
                droneControlSequence[i] = 12;
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

        void GetRemoteControlData()
        {            
            if (remoteControlActual == null || remoteControlTag[0] == null)
            {
                Echo($"Remote Control {antennaTagName} not present");
                return;
            }
            String[] remoteGpsCommand = remoteControlActual.CustomData.Split(':');

            if (remoteGpsCommand.Length < 6)
            {
                remoteControlCustomData1 = "";
                remoteControlCustomData2 = "";
                remoteControlCustomData3 = "";
                remoteControlCustomData4 = "";
                remoteControlCustomData5 = "";
                remoteControlCustomData6 = "";
                prospectTargetValid = false;
                return;
            }
            if (remoteGpsCommand.Length > 6)
            {

                //target_gps_coords = new Vector3D(Double.Parse(remoteGpsCommand[2]), Double.Parse(remoteGpsCommand[3]), Double.Parse(remoteGpsCommand[4]));
                prospectTargetValid = true;
                remoteControlCustomData1 = remoteGpsCommand[1];
                remoteControlCustomData2 = remoteGpsCommand[2];
                remoteControlCustomData3 = remoteGpsCommand[3];
                remoteControlCustomData4 = remoteGpsCommand[4];
                remoteControlCustomData5 = remoteGpsCommand[5];
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
            if (remoteGpsCommand.Length < 12 && remoteGpsCommand.Length > 7)
            {
                remoteControlCustomData7 = "";
                remoteControlCustomData8 = "";
                remoteControlCustomData9 = "";
                remoteControlCustomData10 = "";
                remoteControlCustomData11 = "";
                remoteControlCustomData12 = "";
                prospectAlignTargetValid = false;
                return;
            }
            if (remoteGpsCommand.Length > 7 && !prospectAlignTargetValid)
            {
                bool AlignX = false;
                bool AlignY = false;
                bool AlignZ = false;
                
                remoteControlCustomData7 = remoteGpsCommand[7];
                remoteControlCustomData8 = remoteGpsCommand[8];
                remoteControlCustomData9 = remoteGpsCommand[9];
                remoteControlCustomData10 = remoteGpsCommand[10];
                remoteControlCustomData11 = remoteGpsCommand[11];
                remoteControlCustomData12 = remoteGpsCommand[12];
                if (!double.TryParse(remoteControlCustomData10, out alignGPSCoordinates.X))
                {
                    alignGPSCoordinates.X = 0.0;
                    remoteControlCustomData10 = "";
                    AlignX = false;
                }
                else
                {
                    AlignX = true;
                }
                if (!double.TryParse(remoteControlCustomData11, out alignGPSCoordinates.Y))
                {
                    alignGPSCoordinates.Y = 0.0;
                    remoteControlCustomData11 = "";
                    AlignY = false;
                }
                else
                {
                    AlignY = true;
                }
                if (!double.TryParse(remoteControlCustomData12, out alignGPSCoordinates.Z))
                {
                    alignGPSCoordinates.Z = 0.0;
                    remoteControlCustomData12 = "";
                    AlignZ = false;
                }
                else
                {
                    AlignZ = true;
                }
                if(AlignX && AlignY && AlignZ)
                {
                    prospectAlignTargetValid = true;
                }
            }
        }

        void GetCustomDataJobCommand()
        {

            String[] gpsCommand = Me.CustomData.Split(':');

            if (gpsCommand.Length < 10)
            {
                customData1 = "";
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
                customData16 = "";
                customData17 = "";
                customData18 = "";
                customData19 = "";
                customData20 = "";
                customData21 = "";
                Echo("Data format invalid - GPS:name:x:y:z:depth:grid:numx:numy:limit=True/False:flightfactor:flighthardlimit:skipboresnum");
                return;
            }
            if (gpsCommand.Length > 4)
            {
                bool mAlignX;
                bool mAlignY;
                bool mAlignZ;
                customData1 = gpsCommand[1];
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
            if (gpsCommand.Length > 5)
            {
                customData6 = gpsCommand[6];
                if (!Double.TryParse(customData6, out drillLength))
                {
                    drillLength = 1.0;
                    customData6 = "";
                }
            }
            if (gpsCommand.Length > 6)
            {
                customData7 = gpsCommand[7];
                if (!Double.TryParse(customData7, out gridSize))
                {
                    gridSize = 0.0;
                    customData7 = "";
                }
            }
            if (gpsCommand.Length > 7)
            {
                customData8 = gpsCommand[8];
                if (!int.TryParse(customData8, out numPointsX))
                {
                    numPointsX = 0;
                    customData8 = "";
                }
            }
            if (gpsCommand.Length > 8)
            {
                customData9 = gpsCommand[9];

                if (!int.TryParse(customData9, out numPointsY))
                {
                    numPointsY = 0;
                    customData9 = "";
                }
            }
            if (gpsCommand.Length > 9)
            {
                customData10 = gpsCommand[10];
                if (!Double.TryParse(customData10, out ignoreDepth))
                {
                    ignoreDepth = 0.0;
                    customData10 = "";
                }
            }
            if (gpsCommand.Length > 10)
            {
                customData11 = gpsCommand[11];
                if (!bool.TryParse(customData11, out dronesLaunchedStatus))
                {
                    dronesLaunchedStatus = false;
                    customData11 = "";
                }
            }
            if (gpsCommand.Length > 11)
            {
                customData12 = gpsCommand[12];
                if (!int.TryParse(customData12, out dronesInFlightFactor))
                {
                    dronesInFlightFactor = 1;
                    customData12 = "";
                }
            }
            if (gpsCommand.Length > 12)
            {
                customData13 = gpsCommand[13];
                if (!int.TryParse(customData13, out dronesActiveHardLimit))
                {
                    dronesActiveHardLimit = 6;
                    customData13 = "";
                }
            }
            if (gpsCommand.Length > 13)
            {
                customData14 = gpsCommand[14];
                if (!int.TryParse(customData14, out skipBoresNumber))
                {
                    skipBoresNumber = 0;
                    customData14 = "";
                }
            }
            if (gpsCommand.Length > 14)
            {
                customData15 = gpsCommand[15];
                if (!bool.TryParse(customData15, out coreOutGrid))
                {
                    coreOutGrid = false;
                    customData15 = "";
                }
            }

            if (gpsCommand.Length > 15 && gpsCommand.Length < 24 && !prospectAlignTargetValid)
            {
                Echo($"gpsCommandLen:{gpsCommand.Length}");
                bool targetAlignX;
                bool targetAlignY;
                bool targetAlignZ;
                if (gpsCommand.Length > 15)
                {
                    customData16 = gpsCommand[16];
                }
                if (gpsCommand.Length > 16)
                {
                    customData17 = gpsCommand[17];
                }
                if (gpsCommand.Length > 17)
                {
                    customData18 = gpsCommand[18];
                }
                if (gpsCommand.Length > 18)
                {
                    customData19 = gpsCommand[19];
                }
                if (gpsCommand.Length > 19)
                {
                    customData20 = gpsCommand[20];
                }
                if (gpsCommand.Length > 20)
                {
                    customData21 = gpsCommand[21];
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
                    prospectAlignTargetValid = true;
                }
                else
                {
                    prospectAlignTargetValid = false;
                }
            }
        }

        public void scrnbldr(int ivl, int ivl2, bool slu)
        {
            // Pre-build strings into cl/cl2 for padding
            cl[0] = $"{droneName[ivl]} Status: {droneDamageState[ivl]} {droneControlStatus[ivl]}";
            cl[1] = $"{droneName[ivl]} Docked: {droneDocked[ivl]}";
            cl[2] = $"{droneName[ivl]} Undocked: {droneUndocked[ivl]}";
            cl[3] = $"{droneName[ivl]} Finished: {droneTunnelFinished[ivl]}";
            cl[4] = $"{droneName[ivl]} Mining: {droneMining[ivl]}";
            cl[5] = $"{droneName[ivl]} Waiting: {droneMustWait[ivl]} Reset: {droneResetFunction[ivl]}";
            cl[6] = $"Charge: {drone_charge_storage[ivl]}% Tank: {drone_gas_storage[ivl]}% Cargo: {drone_ore_storage[ivl]}%";
            cl[7] = $"Drill depth: {droneBoreDepth[ivl]}m Start: {drone_mine_depth_start_status[ivl]}m";
            cl[8] = $"Current depth: {droneBoreDepthCurrent[ivl]}m";
            cl[9] = $"Drone control seq: {droneControlSequence[ivl]} Recall seq: {droneRecallSequence[ivl]} {droneRecallList[ivl]}";
            cl[10] = $"Location: {droneGPSListPosition[ivl]} Asnd: {droneAssignedCoordinates[ivl]} Unit OK: {dst[ivl]}";
            cl[11] = $"X: {drone_location_x[ivl]} Y: {drone_location_y[ivl]} Z: {drone_location_z[ivl]}";

            if (slu)
            {
                cl2[0] = $"{droneName[ivl2]} Status: {droneDamageState[ivl2]} {droneControlStatus[ivl2]}";
                cl2[1] = $"{droneName[ivl2]} Docked: {droneDocked[ivl2]}";
                cl2[2] = $"{droneName[ivl2]} Undocked: {droneUndocked[ivl2]}";
                cl2[3] = $"{droneName[ivl2]} Finished: {droneTunnelFinished[ivl2]}";
                cl2[4] = $"{droneName[ivl2]} Mining: {droneMining[ivl2]}";
                cl2[5] = $"{droneName[ivl2]} Waiting: {droneMustWait[ivl2]} Reset: {droneResetFunction[ivl2]}";
                cl2[6] = $"Charge: {drone_charge_storage[ivl2]}% Tank: {drone_gas_storage[ivl2]}% Cargo: {drone_ore_storage[ivl2]}%";
                cl2[7] = $"Drill depth: {droneBoreDepth[ivl2]}m Start: {drone_mine_depth_start_status[ivl2]}m";
                cl2[8] = $"Current depth: {droneBoreDepthCurrent[ivl2]}m";
                cl2[9] = $"Drone control seq: {droneControlSequence[ivl2]} Recall seq: {droneRecallSequence[ivl2]} {droneRecallList[ivl2]}";
                cl2[10] = $"Location: {droneGPSListPosition[ivl2]} Asnd: {droneAssignedCoordinates[ivl2]} Unit OK: {dst[ivl2]}";
                cl2[11] = $"X: {drone_location_x[ivl2]} Y: {drone_location_y[ivl2]} Z: {drone_location_z[ivl2]}";
            }

            droneInformation.AppendLine();
            for (int i = 0; i < 12; i++)
            {
                int padding = Math.Max(clbs - cl[i].Length, 0);
                droneInformation.Append(cl[i]).Append(' ', padding);
                if (slu) droneInformation.AppendLine(cl2[i]);
                else droneInformation.AppendLine();
            }
        }

        IEnumerator<bool> GenListDisplay()
        {
            if (!listHeaderGenerated)
            {
                displayTextList.Append("Mining Grid Status" + " - GMDC " + ver);
                displayTextList.Append('\n');
                displayTextList.Append('\n');
                displayTextList.Append("Remaining bores: " + boresRemaining);
                displayTextList.Append('\n');
                listHeaderGenerated = true;
            }

            for (int i = 0; i < gridBoreFinished.Count; i++)
            {
                for (int j = 0; j < droneGPSListPosition.Count; j++)
                {
                    if (!gridBoreOccupied[i])
                    {
                        drone_namer = "";
                    }
                    else if (i == droneGPSListPosition[j])
                    {
                        drone_namer = droneName[j];
                        drone_assigns_count[j]++;
                    }
                    if (drone_assigns_count[j] > 1)
                    {
                        gridBoreOccupied[i] = false;
                    }
                    drone_assigns_count[j] = 0;

                }
                if (!gridBoreFinished[i])
                {
                    displayTextList.Append('\n');
                    displayTextList.Append($"i: {i}  Mining: {gridBoreOccupied[i].ToString()}  Finished: {gridBoreFinished[i].ToString()} Drone: {drone_namer}");

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
                    Data = $"--- Mining Grid Status ---",
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
                    Data = $"Total Bores: {totalMiningRuns} - Remaining:{boresRemaining} - Drones: {totalDronesMining}",
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

                for (int i = 0; i < gridBorePosition.Count; i++)
                {
                    sprite_total++;
                    Vector3D relativePoint = gridBorePosition[i] - centerPoint;
                    double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                    double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                    var CentX = (float)xPlanar;
                    var CentY = (float)yPlanar;
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

                if (droneName.Count > 0)
                {
                    drone_total = 0;
                    for (int i = 0; i < droneName.Count; i++)
                    {
                        double drone_locale_x = 0.0;
                        double drone_locale_y = 0.0;
                        double drone_locale_z = 0.0;
                        drone_total++;
                        if (!double.TryParse(drone_location_x[i], out drone_locale_x))
                        {
                            drone_locale_x = 0.0;
                        }
                        if (!double.TryParse(drone_location_y[i], out drone_locale_y))
                        {
                            drone_locale_y = 0.0;
                        }
                        if (!double.TryParse(drone_location_z[i], out drone_locale_z))
                        {
                            drone_locale_z = 0.0;
                        }

                        Vector3D Drone_point = new Vector3D(drone_locale_x, drone_locale_y, drone_locale_z);

                        Vector3D relativePoint = Drone_point - centerPoint;
                        double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                        double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                        var CentX = (float)xPlanar;
                        var CentY = (float)yPlanar;
                        string Image_drone = "";
                        var bore_colour_drone = new Color();
                        var alpha_val = 1.0f;

                        if (droneControlStatus[i].Contains("Docked") || droneControlStatus[i].Contains("Undocked") || droneControlStatus[i].Contains("Docking") || droneControlStatus[i].Contains("Undocking"))
                        {
                            alpha_val = 0.25f;
                        }
                        else
                        {
                            alpha_val = 1.0f;
                        }
                        if (!droneMining[i])
                        {
                            Image_drone = "Circle";
                            bore_colour_drone = Color.Gray;
                        }
                        if (droneMining[i])
                        {
                            Image_drone = "Circle";

                            if (droneControlStatus[i].Contains("Min"))
                            {
                                bore_colour_drone = Color.Purple;
                            }
                            else if (droneControlStatus[i].Contains("Exit"))
                            {
                                bore_colour_drone = Color.Orange;
                            }
                            else if (droneControlStatus[i].Contains("RTB: Ready"))
                            {
                                bore_colour_drone = Color.Green;
                            }
                            else if (droneControlStatus[i].Contains("Undock"))
                            {
                                bore_colour_drone = Color.Yellow;
                            }
                            else
                            {
                                bore_colour_drone = Color.Navy;
                            }
                            alpha_val = 1.0f;
                        }
                        if (droneDamageState[i] == "DMG")
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
                        if (drone_cargo_full[i].Contains("True") || drone_recharge_request[i].Contains("True"))
                        {
                            if (drone_recharge_request[i].Contains("True") && drone_cargo_full[i].Contains("True"))
                            {
                                bore_colour_drone = Color.White;
                            }
                            else if (drone_recharge_request[i].Contains("True"))
                            {
                                bore_colour_drone = Color.YellowGreen;
                            }
                            else if (drone_cargo_full[i].Contains("True"))
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

                        if (droneControlStatus[i].Contains("Recharg") || droneControlStatus[i].Contains("Unload") || drone_recharge_request[i].Contains("True"))
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
                            Data = $"{droneName[i]}- ({drone_charge_storage[i]}%)",
                            Position = position,
                            RotationOrScale = 0.3f,
                            Size = sizer * 0.5f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER,
                            FontId = "White"
                        };
                        sprites.Add(sprite_name);
                        percent_list_drones = ((double)drone_total / (double)droneName.Count) * 100;
                        spriteCounter++;
                        yield return false;
                    }
                }
                if (droneName.Count == 0)
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
                if (droneName.Count > 0)
                {
                    if (sprite_total == gridBorePosition.Count && drone_total == droneName.Count)
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
            next_gps_crds = gridBorePosition[currentGPSIndex];
        }


        #region drone_status_verification
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
        int CountIntegerValues(List<int> list, int val)
        {
            int truCnt = 0;

            foreach (int value in list)
            {
                if (value == val)
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
        #endregion


        public void droneCommandBuilder(string cdata_1, string xpos, string ypos, string zpos, string cdata_5, string cmdo, string data_6, string idepth, string xpos2, string ypos2, string zpos2)
        {
            const string baseFormat = "GPS:{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:";
            const string astFormat = "GPS:PAD:{0}:{1}:{2}:#FF75C9F1:";

            c.Clear().EnsureCapacity(prospectAlignTargetValid ? 120 : 80); // ~80 chars base, ~40 more if asteroid
            c.AppendFormat(baseFormat, cdata_1, xpos, ypos, zpos, cdata_5, cmdo, data_6, idepth);
            if (prospectAlignTargetValid) c.AppendFormat(astFormat, xpos2, ypos2, zpos2);
        }
        void transmitToDrone()
        {
            IGC.SendBroadcastMessage(tx_chan, droneTranmissionOutput[di], TransmissionDistance.TransmissionDistanceMax);
        }
        void GetStoredData()
        {
            if (!string.IsNullOrEmpty(Storage)) // Check null or empty
            {
                string[] str_data = Storage.Split(';');
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
        void reset_drone_data()
        {
            droneName.Clear();
            droneDamageState.Clear();
            droneTunnelFinished.Clear();
            droneControlStatus.Clear();
            droneDocked.Clear();
            droneUndocked.Clear();
            droneAutopiloting.Clear();
            droneGPSListPosition.Clear();
            droneGPSCoordinates.Clear();
            droneBoreDepth.Clear();
            droneBoreDepthCurrent.Clear();
            drone_mine_depth_start_status.Clear();
            drone_location_x.Clear();
            drone_location_y.Clear();
            drone_location_z.Clear();
            drone_charge_storage.Clear();
            drone_gas_storage.Clear();
            drone_ore_storage.Clear();
            droneMining.Clear();
            droneAssignedCoordinates.Clear();
            droneControlSequence.Clear(); ;
            droneRecallSequence.Clear();
            droneTranmissionOutput.Clear();
            droneReady.Clear();
            droneMustWait.Clear();
            dcs.Clear();
            dst.Clear();
            droneTransmissionStatus.Clear();
            droneRecallList.Clear();
            droneResetFunction.Clear();
            drone_assigns_count.Clear();
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

        public void drone_custom_data_check(string custominfo, int index)
        {
            Echo("Checking for drone config information..");
            String[] temp_id = custominfo.Split(':');
            Echo($"{temp_id.Length}");

            if (temp_id.Length > 0)
            {
                if (temp_id[0] != null)
                {
                    temp_id_name = temp_id[0];
                    drone_tag = temp_id_name;
                    if (temp_id_name == "" || temp_id_name == null)
                    {
                        temp_id_name = drone_tag;
                        Echo($"Resorting to default scout tag {drone_tag}");
                    }
                }
            }
            else
            {
                temp_id_name = drone_tag;
                Echo($"Resorting to default ID#.{drone_tag}");
            }
            if (temp_id.Length > 1)
            {
                if (temp_id[1] != null)
                {
                    temp_id_name_2 = temp_id[1];
                    secondary = temp_id_name_2;
                    if (temp_id_name_2 == null)
                    {
                        temp_id_name_2 = secondary;
                        Echo($"Resorting to default scout tag {secondary}");
                    }
                }
            }
            else
            {
                temp_id_name_2 = secondary;
                Echo($"Resorting to default ID#.{drone_tag}");
            }
            if (temp_id.Length == 0)
            {
                temp_id_name = drone_tag;
                temp_id_name_2 = secondary;
                Echo($"Resorting to default config {temp_id_name} {temp_id_name_2}.");
            }


            if (antennaAll[index] != null)
            {
                antennaAll[index].CustomData = $"{drone_tag}:{secondary}:";
            }
            Echo($"Drone info:{drone_tag}");
            antennaTagName = "[" + drone_tag + " " + comms + "]";
            lightsTagName = "[" + drone_tag + " " + comms + "]";
            dp_mn_tag = "[" + drone_tag + " " + MainS + " " + dspy + "]";
            dp_drn_tag = "[" + drone_tag + " " + DroneS + " " + dspy + "]";
            dp_lst_tag = "[" + drone_tag + " " + LstS + " " + dspy + "]";
            dp_vis_tag = "[" + drone_tag + " " + GrphS + " " + dspy + "]";
            interfaceTag = "[" + drone_tag + " " + IntfS + "]";
            secondary_tag = "[" + secondary + "]";
            if (secondary == "" || secondary == " " || secondary == null)
            {
                secondary_tag = "";
            }
            rxChannelDrone = drone_tag + " " + replyC;
            rxChannelProspector = drone_tag + " " + prospC;
            tx_recall_channel = drone_tag + " " + commandRecall;
            txDronePingChannel = "[" + drone_tag + "]" + " " + pingMessage;
            Me.CustomName = $"GMDC Programmable Block {secondary_tag} {antennaTagName}";
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
                    drone_custom_data_check(checker, i);
                    if (drone_tag == "" || drone_tag == null)
                    {
                        Echo($"Invalid name for drone_tag {drone_tag} please add drone tag to GMDC antenna custom data '<yourdronetaghere>:<Yourshiptaghere>:' e.g. 'SWRM_D:Atlas:'");
                        return;
                    }
                    antennaAll[i].CustomName = $"GMDC Antenna {secondary_tag} {antennaTagName}";
                    antennaTag.Add(antennaAll[i]);
                }
            }
            antennaAll.Clear();
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
                Echo($"Panel:'{srfV}' on '{dp_vis_tag}' not found");
            }
            if (display_tag_vis.Count <= 0 || display_tag_vis[0] == null)
            {
                Echo($"Display with tag '{dp_vis_tag}' not found");
                Visport_OK = false;
            }
            if (display_tag_vis.Count > 0 && display_tag_vis[0] != null)
            {
                Echo($"Display with tag '{dp_vis_tag}' found");
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
                Echo($"Antenna with tag: '{antennaTagName}' not found.");
                return;
            }
            antennaActual = antennaTag[0];
            if (remoteControlTag.Count <= 0 || remoteControlTag[0] == null)
            {
                Echo($"remote control with tag: '{antennaTagName}' not found.");
                return;
            }
            remoteControlActual = remoteControlTag[0];


            if (lightsTag.Count <= 0 || lightsTag[0] == null)
            {
                Echo($"Indicator light with tag: '{lightsTagName}' not found.");
                return;
            }
            lightIndicatorActual = lightsTag[0];
            lightIndicatorActual.SetValue("Color", Cred);
            if (display_tag_main.Count <= 0 || display_tag_main[0] == null)
            {
                Echo($"Display with tag '{dp_mn_tag}' not found");
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
                Echo($"Panel:'{srfM}' on '{dp_mn_tag}' not found");
            }
            if (display_tag_list.Count <= 0 || display_tag_list[0] == null)
            {
                Echo($"Display with tag '{dp_mn_tag}' not found");
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
                Echo($"Panel:'{srfL}' on '{dp_lst_tag}' not found");
            }
            if (display_tag_drone.Count <= 0 || display_tag_drone[0] == null)
            {
                Echo($"Display with tag '{dp_lst_tag}' not found");
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
                Echo($"Panel:'{srfD}' on '{dp_drn_tag}' not found");
            }
            if (interfacePBTag.Count <= 0 || interfacePBTag[0] == null)
            {
                Echo($"Interface PB with tag: '{interfaceTag}' not found.");
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
                Echo($"Panel:'{srfV}' on '{dp_vis_tag}' not found");
            }
            if (display_tag_vis.Count <= 0 || display_tag_vis[0] == null)
            {
                Echo($"Display with tag '{dp_vis_tag}' not found");
                Visport_OK = false;
            }
            #endregion
        }

        public void ProcessReceivedDroneMessageToDroneLists()
        {
            #region drone_message_data_processing
            if (!string.IsNullOrEmpty(receivedDroneName))
            {
                found = false;
                //drone does not exist assume none and add first
                if (droneName.Count <= 0)
                {
                    droneName.Add(receivedDroneName);
                    droneDamageState.Add(receivedDroneDamageStatus);
                    droneTunnelFinished.Add(receivedDroneTunnelFinished);
                    droneControlStatus.Add(receivedDroneStatus);
                    droneDocked.Add(receivedDroneDocked);
                    droneUndocked.Add(receivedDroneUndocked);
                    droneAutopiloting.Add(recived_drone_autopilot);
                    droneGPSListPosition.Add(-1);
                    droneGPSCoordinates.Add(remoteControlActual.GetPosition());
                    droneBoreDepth.Add(rc_dn_drl_dpth);
                    droneBoreDepthCurrent.Add(rc_dn_drl_crnt);
                    drone_mine_depth_start_status.Add(rc_dn_drl_strt);
                    drone_location_x.Add(rc_locx);
                    drone_location_y.Add(rc_locy);
                    drone_location_z.Add(rc_locz);
                    drone_charge_storage.Add(rc_dn_chg);
                    drone_gas_storage.Add(rc_dn_gas);
                    drone_ore_storage.Add(rc_dn_str);
                    droneMining.Add(false);
                    droneAssignedCoordinates.Add(false);
                    droneControlSequence.Add(0);
                    droneRecallSequence.Add(0);
                    droneTranmissionOutput.Add("");
                    droneReady.Add(false);
                    droneMustWait.Add(true);
                    dcs.Add(0.0);
                    dst.Add(true);
                    droneTransmissionStatus.Add(true);
                    droneRecallList.Add(false);
                    droneResetFunction.Add(false);
                    drone_assigns_count.Add(0);
                    drone_cargo_full.Add(rc_dn_cargo_full);
                    drone_recharge_request.Add(rc_dn_rchg_req);
                    drone_auto_pilot_enabled.Add(rc_auto_pilot_enabled);
                    droneAutodock.Add(recievedDroneAutdock);
                    droneDockingReady.Add(recievedDroneDockingReady);
                }
                //drones do exist so check current drone list to see if it exists
                if (droneName.Count > 0)
                {

                    for (int i = 0; i < droneName.Count; i++)
                    {
                        found = false;
                        string nametag = droneName[i];
                        //drone name found so update data
                        if (nametag == receivedDroneName)
                        {
                            found = true;
                            droneName[i] = receivedDroneName;
                            droneDamageState[i] = receivedDroneDamageStatus;
                            droneTunnelFinished[i] = receivedDroneTunnelFinished;
                            droneControlStatus[i] = receivedDroneStatus;
                            droneDocked[i] = receivedDroneDocked;
                            droneUndocked[i] = receivedDroneUndocked;
                            droneAutopiloting[i] = recived_drone_autopilot;
                            droneBoreDepth[i] = (rc_dn_drl_dpth);
                            droneBoreDepthCurrent[i] = rc_dn_drl_crnt;
                            drone_mine_depth_start_status[i] = rc_dn_drl_strt;
                            droneGPSListPosition[i] = recieved_drone_list_position;
                            drone_location_x[i] = rc_locx;
                            drone_location_y[i] = rc_locy;
                            drone_location_z[i] = rc_locz;
                            drone_charge_storage[i] = rc_dn_chg;
                            drone_gas_storage[i] = rc_dn_gas;
                            drone_ore_storage[i] = rc_dn_str;
                            dcs[i] = rc_d_cn;
                            droneTransmissionStatus[i] = true;
                            drone_cargo_full[i] = rc_dn_cargo_full;
                            drone_recharge_request[i] = rc_dn_rchg_req;
                            drone_auto_pilot_enabled[i] = rc_auto_pilot_enabled;
                            droneAutodock[i] = recievedDroneAutdock;
                            droneDockingReady[i] = recievedDroneDockingReady;
                            if (dcs[i] <= bclm)
                            {
                                dst[i] = false;
                            }
                            if (dcs[i] > bclm)
                            {
                                dst[i] = true;
                            }
                            droneMessageConfirmed = true;
                            receivedDroneNameIndex = i;
                            break;
                        }
                        //drone not found at end of list so add drone to list
                        if (i == (droneName.Count - 1) && !nametag.Equals(receivedDroneName) && !found)
                        {
                            droneName.Add(receivedDroneName);
                            droneDamageState.Add(receivedDroneDamageStatus);
                            droneTunnelFinished.Add(receivedDroneTunnelFinished);
                            droneControlStatus.Add(receivedDroneStatus);
                            droneDocked.Add(receivedDroneDocked);
                            droneUndocked.Add(receivedDroneUndocked);
                            droneAutopiloting.Add(recived_drone_autopilot);
                            droneGPSCoordinates.Add(remoteControlActual.GetPosition());
                            droneBoreDepth.Add(rc_dn_drl_dpth);
                            droneBoreDepthCurrent.Add(rc_dn_drl_crnt);
                            drone_mine_depth_start_status.Add(rc_dn_drl_strt);
                            drone_location_x.Add(rc_locx);
                            drone_location_y.Add(rc_locy);
                            drone_location_z.Add(rc_locz);
                            drone_charge_storage.Add(rc_dn_chg);
                            drone_gas_storage.Add(rc_dn_gas);
                            drone_ore_storage.Add(rc_dn_str);
                            droneMining.Add(false);
                            droneAssignedCoordinates.Add(false);
                            droneGPSListPosition.Add(-1);
                            droneControlSequence.Add(0);
                            droneRecallSequence.Add(0);
                            droneTranmissionOutput.Add("");
                            droneReady.Add(false);
                            droneMustWait.Add(true);
                            dcs.Add(0.0);
                            dst.Add(true);
                            droneTransmissionStatus.Add(true);
                            droneRecallList.Add(false);
                            droneResetFunction.Add(false);
                            drone_assigns_count.Add(0);
                            drone_cargo_full.Add(rc_dn_cargo_full);
                            drone_recharge_request.Add(rc_dn_rchg_req);
                            drone_auto_pilot_enabled.Add(rc_auto_pilot_enabled);
                            droneAutodock.Add(recievedDroneAutdock);
                            droneDockingReady.Add(recievedDroneDockingReady);
                            droneMessageConfirmed = true;
                            receivedDroneNameIndex = i + 1;
                        }
                    }
                }
            }
            #endregion
        }

        void update_display()  // Extracted from drone_processing
        {
            displayTextMain.Clear().EnsureCapacity(512); // ~400-600 chars typical
            displayTextMain.AppendLine($"GMDC {ver} Running {icon}")
                  .AppendLine($"------------------------------")
                  .AppendLine($" ")
                   .AppendLine($"Total drones detected: {droneName.Count}")
                  .AppendLine(dronesLaunchedStatus
                      ? $"Drones active: {totalDronesMining} Idle: {t_drn_idle} Fault: {totalDronesDamaged} (Max: {maxActiveDronesCount} ({dronesInFlightFactor})) Hard limit: {dronesActiveHardLimit}"
                      : $"Drones active: {totalDronesMining} Idle: {t_drn_idle} Fault: {totalDronesDamaged}")
                  .AppendLine($"Docking: {t_drn_dckg} Docked: {t_drn_dck} - Recharge: {t_drn_rechg} Unload: {t_drn_unload}")
                  .AppendLine($"Undocking: {t_drn_udckg} Undocked: {t_drn_udck} - Mining: {t_drn_mine} Exit: {t_drn_exit}")
                  .AppendLine()
                  .AppendLine($"Surface distance: {safe_dstvl}m")
                  .AppendLine($"Drill depth: {drillLength}m ({drillLength + safe_dstvl}m)")
                  .AppendLine($"Req. ignore depth: {ignoreDepth}m (Drone length: {drone_length}m)")
                  .AppendLine($"Ignore depth: {safe_dstvl + drone_length + ignoreDepth}m (Drill Start: {(drillLength + safe_dstvl) - (ignoreDepth + safe_dstvl + drone_length)}m)")
                  .AppendLine()
                  .AppendLine($"Command: {commandAsk} Reset: {generalReset}")
                  .AppendLine($"Status: {screenStatus}")
                  .AppendLine()
                  .AppendLine("Target:")
                  .AppendLine(miningGPSCoordinates.ToString());
            if (prospectAlignTargetValid) displayTextMain.AppendLine("Secondary/Asteroid:").AppendLine(alignGPSCoordinates.ToString());

            if (display_tag_main.Count > 0 && sM != null) sM.WriteText(displayTextMain);
        }

        //program end

    }
}

