using Sandbox.Game.EntityComponents;
using Sandbox.Game.GameSystems.Conveyors;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography;
using System.Security.Policy;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Utils;
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
        int undock_delay_time = 60;
        int undock_delay_limit = 120;

        //Drone Comms
        int drone_comms_processing_delay = 1;
        int drone_ping_time_delay = 18;

        #endregion

        #region static_variables
        //visualiser settings
        int spritecount_limit_main = 500;
        int spritecount_limit_insert = 250;
        //statics
        int game_factor = 10;
        string ver = "V0.385";
        string comms = "Comms";
        string MainS = "Main";
        string DroneS = "Drone";
        string IntfS = "Interface";
        string LstS = "List";
        string dspy = "Display";
        string GrphS = "Visual";
        bool core_out = false;
        int clbs = 44;
        double bclu = 30.0;
        string tx_chan;
        bool launched_drone_status = false;
        int flight_factor = 1;
        int hard_active_drone_limit = 10;
        int skip_bores_number = 0;
        double gridSize;
        int nPtsY;
        int nPtsX;

        bool created_grid = false;
        int max_active_drone_count;
        string dataMessageFromDrone;
        string data_in_prospector;
        double bclm = 1.0;
        string recieved_drone_id_name;
        string rc_dds;
        string rc_tnl_end;
        string rc_dn_sts;
        string recieved_drone_dock;
        string recieved_drone_undock;
        string recived_drone_autopilot;
        string rc_dn_drl_dpth;
        string rc_dn_drl_crnt;
        string rc_dn_drl_strt;
        string rc_dn_gps_lst;
        string rc_dn_cargo_full;
        string rc_dn_rchg_req;
        string rc_auto_pilot_enabled;
        int recieved_drone_list_position;
        double rc_d_cn = 0.0;
        string rc_locx;
        string rc_locy;
        string rc_locz;
        string rc_dn_chg;
        string rc_dn_gas;
        string rc_dn_str;
        int current_gps_idx = 0;
        int r_gps_idx = 0;
        string rx_ch = "";
        string rx_ch_2 = "";
        string tx_recall_channel = "";
        string tx_drone_recall_channel = "";
        string tx_ping_channel = "";
        string p_cht = "ping";
        bool pinged = false;
        string ant_tg = "";
        string lt_tag = "";
        string dp_mn_tag = "";
        string dp_drn_tag = "";
        string dp_lst_tag = "";
        string dp_vis_tag = "";
        string intfc_tag = "";
        double drillLength;
        bool dat_vld = false;
        bool can_run = false;
        bool canReset = false;
        bool canTransmit = false;
        bool mustRecall_Command = false;
        bool mustFreeze_Command = false;
        bool can_init = false;
        bool found = false;
        bool general_reset;
        bool mining_grid_valid = false;
        List<bool> drone_must_wait;
        double ignoreDepth = 0.0;
        double safe_dstvl = 0.0;
        bool target_valid = false;
        bool align_target_valid = false;
        string commandAsk;
        string cst_dt1;
        string cst_dt2;
        string cst_dt3;
        string cst_dt4;
        string cst_dt5;
        string cst_dt6;
        string cst_dt7;
        string cst_dt8;
        string cst_dt9;
        string cst_dt10;
        string cst_dt11;
        string cst_dt12;
        string cst_dt13;
        string cst_dt14;
        string cst_dt15;
        string rm_cst_dat1 = "";
        string rm_cst_dat2 = "";
        string rm_cst_dat3 = "";
        string rm_cst_dat4 = "";
        string rm_cst_dat5 = "";
        string rm_cst_dat6 = "";
        string rm_cst_dat7 = "";
        string rm_cst_dat8 = "";
        string rm_cst_dat9 = "";
        string rm_cst_dat10 = "";
        string rm_cst_dat11 = "";
        string rm_cst_dat12 = "";
        string currentDroneBoreIndex = "";
        string currentMiningDroneBoreLocationX = "";
        string currentMiningDroneBoreLocationY = "";
        string currentMiningDroneBoreLocationZ = "";
        string currentMiningDroneCustomData_5 = "";
        string commandOutput = "";
        string currentMiningDroneBoreLength = "";
        string currentMiningDroneIgnoreDistance = "";
        string currentMiningDroneAlignLocationX = "";
        string currentMiningDroneAlignLocationY = "";
        string currentMiningDroneAlignLocationZ = "";
        int t_mne_sq_cmp = 0;
        int total_mining_runs = 1;
        int total_drones_mining = 0;
        int t_drn_dckg = 0;
        int t_drn_dck = 0;
        int t_drn_udckg = 0;
        int t_drn_udck = 0;
        int t_drn_rechg = 0;
        int t_drn_unload = 0;
        int t_drn_idle = 0;
        int t_drn_exit = 0;
        int t_drn_mine = 0;
        int bores_remaining;
        bool faultOutput = false;
        int faultCount = 0;
        private IEnumerator<bool> gridCoroutine;
        private IEnumerator<bool> listCoroutine;
        private IEnumerator<bool> visCoroutine;
        IMyRadioAntenna ant_act;
        IMyLightingBlock light_status_indicator_actual;
        IMyRemoteControl remote_control_actual;
        IMyProgrammableBlock pb_i_act;
        Vector3D mainGPSCoordinates;
        Vector3D centralGridGPSCoordinates;
        Vector3D next_gps_crds;
        Vector3D target_gps_coords;
        Vector3D alignGPSCoordinates;
        Vector3D planeNrml;
        StringBuilder miningCoordinatesNew;
        StringBuilder c;
        List<Vector3D> drone_location;
        List<string> drone_name;
        List<string> drone_damage_state;
        List<string> drone_tunnel_complete;
        List<string> drone_control_status;
        List<string> drone_dock_status;
        List<string> drone_undock_status;
        List<string> drone_autopilot_status;
        List<string> drone_drill_depth_value;
        List<string> drone_mine_distance_status;
        List<string> drone_mine_depth_start_status;
        List<int> drone_gps_grid_list_position;
        List<bool> drone_ready;
        List<string> drone_location_x;
        List<string> drone_location_y;
        List<string> drone_location_z;
        List<string> drone_charge_storage;
        List<string> drone_gas_storage;
        List<string> drone_ore_storage;
        List<string> drone_cargo_full;
        List<string> drone_recharge_request;
        List<string> drone_auto_pilot_enabled;
        List<int> drone_assigns_count;
        List<double> dcs;
        List<bool> drone_assigned_coordinates;
        List<bool> drone_recall_list;
        List<Vector3D> drone_gps_coordinates_ds;
        List<int> drone_control_sequence;
        List<int> drone_recall_sequence;
        List<bool> drone_reset_func;
        List<string> droneTranmissionOutput;
        List<GridBoreData> gridBoreDataList;
        List<DroneData> droneDataList;
        List<string> cl;
        List<string> cl2;
        List<int> tla;
        List<int> rst;
        List<string> fct;
        List<bool> droneStatusOK;
        List<bool> droneTransmissionStatus;
        //int cbval = 0;
        //bool clbt = false;
        int bores_completed;
        int gps_grid_position_value = -1;
        string drone_namer = "";
        StringBuilder droneInformation;
        StringBuilder dp_txm;
        StringBuilder dp_txl;
        StringBuilder jxt;
        List<bool> drone_mining;
        bool setup_complete = false;
        bool time_delay = false;
        int time_count = 0;
        int pngt_count = 0;
        bool readyFlag = false;
        int reset_status_count = 0;
        int docked_status_count = 0;
        string stts = "Idle";
        string replyC = "reply";
        string prospC = "prospector";
        string command_recall = "recall";
        string command_operate = "operate";
        bool mustUndock_Command = false;
        bool run_arg = false;
        bool renew_header = true;
        Color Cgreen = new Color(0, 255, 0);
        Color Cyellow = new Color(255, 255, 0);
        Color Cred = new Color(255, 0, 0);
        Color Cblue = new Color(0, 0, 255);
        Color Coren = new Color(235, 90, 33);
        bool can_intf = false;
        bool i_init = false;
        bool i_res = false;
        bool i_run = false;
        bool i_recall = false;
        bool i_eject = false;
        bool i_frz = false;
        bool i_stop = false;
        bool n_intf = false;
        string it_ag;
        List<IMyRemoteControl> rm_ctl_all;
        List<IMyRemoteControl> rm_ctl_tag;
        List<IMyRadioAntenna> at_all;
        List<IMyRadioAntenna> at_tg;
        List<IMyLightingBlock> lts_all;
        List<IMyLightingBlock> lts_sys_tg;
        List<IMyTerminalBlock> display_all;
        List<IMyTerminalBlock> display_tag_main;
        List<IMyTerminalBlock> display_tag_list;
        List<IMyTerminalBlock> display_tag_drone;
        List<IMyTerminalBlock> display_tag_vis;
        List<IMyProgrammableBlock> pb_all;
        List<IMyProgrammableBlock> pb_tg;
        IMyTextSurface sD;
        IMyTextSurface sM;
        IMyTextSurface sL;
        IMyTextSurface sV;
        RectangleF _viewport;
        StringBuilder sb;
        
        int t_drn_dmg = 0;
        int t_dn_unk = 0;
        int t_dn_ok = 0;
        int di = 0;
        int total_drones_undocking = 0;
        int undock_timer = 0;
        bool drones_undocking = false;
        bool can_loading = false;
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        int initgridcount = 0;
        bool init_grid_complete = false;
        int debugcount = 0;
        bool bores_regen;
        bool listgenerator_finished = false;
        bool listheader_generated = false;
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

        IMyBroadcastListener listen;
        IMyBroadcastListener listen_prspt;
        List<MyIGCMessage> drone_messages_list;
        List<MyIGCMessage> prospector_messages_list;
        bool Prospect_Message = false;
        bool Confirmed_Drone_Message = false;
        int recieved_drone_name_index = -1;
        bool Drone_Message = false;
        bool Visport_OK = false;
        List<MySprite> sprites;
        int spritecount = 0;
        bool sprite_insert = false;
        StringBuilder customDataString;

        private double totalRuntimeMs = 0.0;
        private int runCount = 0;
        private double averageRuntimeMs = 0.0;
        #endregion
        public void Save()
        {
            if (!setup_complete || gridBoreDataList == null) return; // Early exit if not ready

            sb.Clear(); // Reuse existing StringBuilder
            for (int i = 0; i < gridBoreDataList.Count; i++)
            {
                var bore = gridBoreDataList[i];
                sb.AppendFormat("{0}:{1}:{2}:{3}:{4};",
                    bore.boreFinished ? "1" : "0",
                    bore.boreOccupied ? "1" : "0",
                    bore.borePosition.X,
                    bore.borePosition.Y,
                    bore.borePosition.Z);
            }
            Storage = sb.ToString();
            sb.Clear();
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
            if (!setup_complete)
            {
                setup_system();
                setup_complete = true;
                Echo("Setup complete!");
            }
            CheckSystemStatus();
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
            listen = IGC.RegisterBroadcastListener(rx_ch);
            listen_prspt = IGC.RegisterBroadcastListener(rx_ch_2);
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
            Echo($"Pre-Prospect: valid={target_valid}, coords={target_gps_coords}");
            if (Prospect_Message)
            {
                Storage = null;
                GetRemoteControlData();
                Echo($"Post-Prospect: valid={target_valid}, coords={target_gps_coords}");
                if (target_valid)
                {
                    Echo($"Formatting CustomData with: {target_gps_coords.X}, {target_gps_coords.Y}, {target_gps_coords.Z}");
                    miningCoordinatesNew.Clear().AppendFormat("GPS:PDT:{0:0.##}:{1:0.##}:{2:0.##}:#FF75C9F1:5.0:10.0:1:1:0:False:1:10:0:",
                        target_gps_coords.X, target_gps_coords.Y, target_gps_coords.Z);
                    Me.CustomData = miningCoordinatesNew.ToString();
                }
                Prospect_Message = false;
                created_grid = false;
            }
            GetCustomData_JobCommand();
            ProcessJobGrid();
            UpdateActiveDroneLimits();
            Echo($"UpdateMiningGrid: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void HandleDroneOperations()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (drone_name.Count > 0 && created_grid && time_delay)
            {
                UpdateDroneCounts();
                DroneUndockCheck();
                timeCountReset();
                ProcessRecallCommand();
                ProcessDroneState();
                update_display();
            }
            drone_reset_status_counter();
            indication_and_status_management();
            Echo($"HandleDroneOperations: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void RenderDisplays()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            drone_render_call();
            list_render_call();
            sprite_render_call();
            Echo($"RenderDisplays: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateStatus()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            time_counter_reset();
            Local_Status_Update(Runtime.LastRunTimeMs);
            Echo($"UpdateStatus: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void Local_Status_Update(double _Runtime)
        {
            Echo($"Load: {Math.Round((_Runtime / game_tick_length) * (double)100.0, 3)}% ({Math.Round(_Runtime, 3)}ms) S#:{spritecount} {sprite_insert}");
            Echo($"Drones #: {drone_name.Count}");
            Echo($"Drone comms buffer: {drone_messages_list.Count} OK: {Drone_Message}");
            Echo($"Cycles since last broadcast: {time_count} ({Math.Round((((double)drone_comms_processing_delay * game_tick_length) / (double)1000) * (double)game_factor, 1)}s) {time_delay}");
            Echo($"Cycles since last ping: {pngt_count} ({Math.Round((((double)drone_ping_time_delay * game_tick_length) / (double)1000) * (double)game_factor, 1)}s)");
            Echo($"Undock cycle timer: {undock_timer} ({Math.Round((((double)undock_timer * game_tick_length) / (double)1000) * (double)game_factor, 1)}s) ({Math.Round((((double)undock_delay_limit * game_tick_length) / (double)1000) * (double)game_factor, 1)}s)");
            Echo($"Drones Undocking: {drones_undocking} {total_drones_undocking}");
            Echo($"Prospect comms buffer: {prospector_messages_list.Count}");
            state_shifter();
            //debugger            
            //Echo($"{init_grid_complete} {c_gd}");

        }

        private void time_counter_reset()
        {
            time_count++;
            if (time_count >= drone_comms_processing_delay)
            {
                time_delay = true;
            }
            pngt_count++;
            if (pngt_count >= drone_ping_time_delay)
            {
                pinged = false;
            }
            if (drones_undocking)
            {
                undock_timer++;
            }
            if (undock_timer > undock_delay_time)
            {
                drones_undocking = false;
            }
        }

        private void sprite_render_call()
        {
            //coroutine visual
            if (Visport_OK)
            {


                if (visCoroutine == null && !frame_generator_finished)
                {
                    visCoroutine = BuildSprites(mainGPSCoordinates, planeNrml, gridSize, nPtsX, nPtsY, core_out);
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
                        BuildSprites(mainGPSCoordinates, planeNrml, gridSize, nPtsX, nPtsY, core_out).Dispose();
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
                if (display_tag_vis.Count > 0 && display_tag_vis[0] != null && gridBoreDataList.Count > 0 && frame_generator_finished)
                {
                    frame_generator_finished = false;
                    if (spritecount >= spritecount_limit_main)
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
                        Echo($"Frame reset - spritecount {spritecount}");
                        var frame = sV.DrawFrame();
                        DrawSprites(ref frame);
                        frame.Dispose();
                        sprites.Clear();

                    }
                    if (spritecount > spritecount_limit_main + 1)
                    {
                        spritecount = 0;
                        sprite_insert = false;
                    }

                }
            }
        }

        private void list_render_call()
        {
            //coroutine list
            if (listCoroutine == null && !listgenerator_finished)
            {
                listCoroutine = GenListDisplay();
            }
            if (listCoroutine != null && !listgenerator_finished)
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
            if (display_tag_list.Count > 0 && display_tag_list[0] != null && listgenerator_finished)
            {
                sL.WriteText(dp_txl.ToString());
                listgenerator_finished = false;
                dp_txl.Clear();
                listheader_generated = false;
            }
        }

        private void indication_and_status_management()
        {
            if (canTransmit && !readyFlag || commandAsk == "Init")
            {
                light_status_indicator_actual.SetValue("Color", Cred);
                if (commandAsk == "Init")
                {
                    light_status_indicator_actual.SetValue("Color", Cgreen);
                }
                light_status_indicator_actual.Enabled = true;
                light_status_indicator_actual.BlinkIntervalSeconds = 0.7f;
                light_status_indicator_actual.BlinkLength = 20.0f;
                light_status_indicator_actual.Enabled = true;
                stts = "Not Ready";
            }
            if (canTransmit && readyFlag)
            {
                light_status_indicator_actual.SetValue("Color", Cgreen);
                light_status_indicator_actual.BlinkIntervalSeconds = 0;
                light_status_indicator_actual.BlinkLength = 10.0f;
                light_status_indicator_actual.Enabled = true;
                stts = "Ready";
            }
            if (!canTransmit && commandAsk == "Stop" || commandAsk == "" || commandAsk == "Freeze" || commandAsk == "Eject" || commandAsk == "Recall")
            {
                light_status_indicator_actual.SetValue("Color", Cred);
                if (commandAsk == "Eject")
                {
                    light_status_indicator_actual.SetValue("Color", Cyellow);
                }

                if (commandAsk == "Recall")
                {
                    light_status_indicator_actual.SetValue("Color", Cblue);
                }
                light_status_indicator_actual.BlinkIntervalSeconds = 0.7f;
                light_status_indicator_actual.BlinkLength = 20.0f;
                light_status_indicator_actual.Enabled = true;
                stts = "Not Ready";
            }

            if (total_drones_mining > 0 && bores_completed < total_mining_runs && canTransmit && can_run || faultOutput)
            {
                light_status_indicator_actual.BlinkIntervalSeconds = 0;
                if (t_dn_unk > 0)
                {
                    light_status_indicator_actual.BlinkIntervalSeconds = 0.7f;
                }
                light_status_indicator_actual.SetValue("Color", Cyellow);
                if (t_drn_dmg > 0)
                {
                    light_status_indicator_actual.BlinkIntervalSeconds = 0.7f;
                    light_status_indicator_actual.SetValue("Color", Coren);
                }
                if (faultOutput)
                {
                    light_status_indicator_actual.BlinkIntervalSeconds = 0;
                    light_status_indicator_actual.SetValue("Color", Coren);
                }

                light_status_indicator_actual.Enabled = true;
                stts = "Working";
            }
            if (bores_completed >= total_mining_runs)
            {
                light_status_indicator_actual.SetValue("Color", Cred);
                light_status_indicator_actual.Enabled = true;
                stts = "Sequence Finished";
            }
        }

        private void drone_reset_status_counter()
        {
            if (drone_gps_grid_list_position.Count > 0 && canReset)
            {
                reset_status_count = CntIntVls(drone_gps_grid_list_position, -1);
                docked_status_count = CntStsVls(drone_control_status, "Docked");
            }
            if (drone_name.Count > 0)
            {
                if (reset_status_count == drone_name.Count && docked_status_count == drone_name.Count && canReset)
                {
                    readyFlag = true;
                }
            }
        }

        private void ProcessDroneState()
        {
            #region drone_state_machine_management
            if (recieved_drone_name_index != -1 && Confirmed_Drone_Message)
            {

                int i = recieved_drone_name_index;

                if (can_init || canReset || drone_reset_func[i] || can_loading)
                {
                    general_reset = true;
                }
                else general_reset = false;
                di = i;
                faultCount = CntTrueVls(droneStatusOK);
                if (faultCount < drone_name.Count)
                {
                    faultOutput = true;
                }
                else faultOutput = false;

                //recall sequence reset - global
                if (!drone_recall_list[i] || canReset || can_init || can_loading)
                {
                    drone_recall_sequence[i] = 0;
                }
                dp_txm.Clear();

                if (drone_gps_grid_list_position[i] > -1 && !drone_assigned_coordinates[i])
                {
                    drone_gps_grid_list_position[i] = -1;
                }
                //if undocked request local recall sequence flag to ON
                if (drone_gps_grid_list_position[i] == -1 && !drone_assigned_coordinates[i] && drone_undock_status[i] == "True" && drone_dock_status[i] == "False" && !drone_recall_list[i] && !mustUndock_Command || drone_gps_grid_list_position[i] == -1 && !drone_assigned_coordinates[i] && drone_undock_status[i] == "False" && drone_dock_status[i] == "False" && !drone_recall_list[i] && !mustUndock_Command)
                {
                    drone_recall_list[i] = true;
                }
                if (drone_recall_list[i])
                {
                    tx_drone_recall_channel = drone_name[i] + " " + command_recall;
                    IGC.SendBroadcastMessage(tx_drone_recall_channel, command_recall, TransmissionDistance.TransmissionDistanceMax);
                }
                if (!drone_recall_list[i])
                {
                    tx_drone_recall_channel = drone_name[i] + " " + command_recall;
                    IGC.SendBroadcastMessage(tx_drone_recall_channel, command_operate, TransmissionDistance.TransmissionDistanceMax);
                }


                if (drone_control_status[i].Contains("Docked") && drone_gps_grid_list_position[i] == -1 && drone_mining[i] && drone_control_sequence[i] == 0)
                {
                    drone_mining[i] = false;
                }





                if (total_drones_mining >= bores_remaining && !drone_mining[i] && bores_completed <= total_mining_runs || bores_remaining == 0 && drone_mining[i] == false)
                {
                    if (!launched_drone_status || drones_undocking)
                    {
                        drone_must_wait[i] = true;
                    }
                    if (launched_drone_status && total_drones_mining > max_active_drone_count || drones_undocking)
                    {
                        drone_must_wait[i] = true;
                    }
                    if (launched_drone_status && total_drones_mining <= max_active_drone_count)
                    {
                        drone_must_wait[i] = false;
                    }
                }
                else if (total_drones_mining < bores_remaining && bores_completed < total_mining_runs || drone_mining[i] && total_drones_mining <= bores_remaining)
                {
                    if (!launched_drone_status)
                    {
                        drone_must_wait[i] = false;
                    }
                    if (launched_drone_status && total_drones_mining < max_active_drone_count)
                    {
                        drone_must_wait[i] = false;
                    }

                    if (launched_drone_status && total_drones_mining > max_active_drone_count || drones_undocking)
                    {
                        drone_must_wait[i] = true;
                    }
                }
                if (drone_gps_grid_list_position[i] == -1 && total_drones_mining >= bores_remaining || drones_undocking)
                {
                    if (!launched_drone_status)
                    {
                        drone_must_wait[i] = true;
                    }
                    if (launched_drone_status && total_drones_mining >= max_active_drone_count || drones_undocking)
                    {
                        drone_must_wait[i] = true;
                    }
                    if (launched_drone_status && total_drones_mining < max_active_drone_count || drones_undocking)
                    {
                        drone_must_wait[i] = true;
                    }
                }
                if (drone_gps_grid_list_position[i] > -1 && drone_gps_grid_list_position[i] < gridBoreDataList.Count)
                {
                    if (gridBoreDataList[drone_gps_grid_list_position[i]].boreOccupied && !drone_mining[i])
                    {
                        if (!launched_drone_status || drones_undocking)
                        {
                            drone_must_wait[i] = true;
                        }
                        if (launched_drone_status && total_drones_mining >= max_active_drone_count || drones_undocking)
                        {
                            drone_must_wait[i] = true;
                        }
                        if (launched_drone_status && total_drones_mining < max_active_drone_count || drones_undocking)
                        {
                            drone_must_wait[i] = true;
                        }
                    }
                    else if (bores_completed < total_mining_runs && !gridBoreDataList[drone_gps_grid_list_position[i]].boreOccupied && !gridBoreDataList[drone_gps_grid_list_position[i]].boreFinished && !drone_mining[i])
                    {
                        if (!launched_drone_status)
                        {
                            drone_must_wait[i] = false;
                        }
                        if (launched_drone_status && total_drones_mining < max_active_drone_count)
                        {
                            drone_must_wait[i] = false;
                        }
                        if (launched_drone_status && total_drones_mining >= max_active_drone_count || drones_undocking)
                        {
                            drone_must_wait[i] = true;
                        }
                    }
                    if (!gridBoreDataList[drone_gps_grid_list_position[i]].boreFinished)
                    {
                        int queued_count = CntIntVls(drone_gps_grid_list_position, drone_gps_grid_list_position[i]);
                        if (gridBoreDataList[drone_gps_grid_list_position[i]].boreOccupied && queued_count == 0)
                        {
                            GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                            bore.boreOccupied = false;
                            gridBoreDataList[drone_gps_grid_list_position[i]] = bore;
                        }
                    }
                    else
                    {
                        Echo($"Warning: Invalid bore index {drone_gps_grid_list_position[i]} for drone {i}");
                    }
                }

                updateDisplay(i);

                gps_grid_position_value = drone_gps_grid_list_position[i];
                if (drone_control_status[i] == "Docked Idle")
                {
                    drone_ready[i] = true;
                }
                if (drone_control_status[i].Contains("Recharging") || drone_control_status[i].Contains("Unloading"))
                {
                    drone_ready[i] = false;
                }
                if (drone_ready[i] && drone_tunnel_complete[i] == "False" && drone_dock_status[i] == "True" && can_run && !drone_assigned_coordinates[i] && drone_control_sequence[i] == 0 && !drone_must_wait[i] && !drone_mining[i] && !run_arg)
                {
                    if (bores_completed < total_mining_runs && mining_grid_valid != false && !drone_assigned_coordinates[i] && drone_must_wait[i] == false)
                    {

                        if (gridBoreDataList.Count > 0)
                        {
                            if (skip_bores_number > gridBoreDataList.Count)
                            {
                                skip_bores_number = 0;
                            }
                            if (skip_bores_number > 0)
                            {
                                for (int j = 0; j < skip_bores_number; j++)
                                {
                                    if (j > gridBoreDataList.Count - 1 || j > skip_bores_number - 1)
                                    {
                                        break;
                                    }
                                    GridBoreData bore = gridBoreDataList[j];
                                    bore.boreFinished = true;
                                    gridBoreDataList[j] = bore;
                                }
                            }
                            for (int k = 0; k < gridBoreDataList.Count; k++)
                            {

                                if (k > gridBoreDataList.Count - 1)
                                {
                                    k = gridBoreDataList.Count - 1;
                                }
                                if (!gridBoreDataList[k].boreFinished && !gridBoreDataList[k].boreOccupied)
                                {
                                    current_gps_idx = k;
                                    break;
                                }
                            }
                        }
                        r_gps_idx = current_gps_idx;
                        if (gps_grid_position_value == -1)
                        {
                            gps_grid_position_value = current_gps_idx;
                            drone_gps_coordinates_ds[i] = gridBoreDataList[gps_grid_position_value].borePosition;
                            drone_gps_grid_list_position[i] = gps_grid_position_value;
                        }
                        else
                        {
                            gps_grid_position_value = drone_gps_grid_list_position[i];
                            drone_gps_coordinates_ds[i] = gridBoreDataList[gps_grid_position_value].borePosition;
                        }
                        if (!mining_grid_valid)
                        {
                            total_mining_runs = 1;
                            drone_gps_coordinates_ds[i] = mainGPSCoordinates;
                            gps_grid_position_value = 0;
                            current_gps_idx = 0;
                        }
                        //suspect code here
                        Echo($"Drone coords: {i}");
                        drone_assigned_coordinates[i] = true;
                        Echo($"Drone coords assigned: {i} {drone_assigned_coordinates[i]}");
                    }
                    else if (!mining_grid_valid)
                    {
                        total_mining_runs = 1;
                        drone_gps_coordinates_ds[i] = mainGPSCoordinates;
                        drone_assigned_coordinates[i] = true;
                        gps_grid_position_value = 0;
                        current_gps_idx = 0;
                    }
                    if (drone_gps_grid_list_position[i] > -1)
                    {
                        if (gridBoreDataList[drone_gps_grid_list_position[i]].boreOccupied && !drone_mining[i])
                        {
                            drone_must_wait[i] = true;
                        }
                        else if (total_drones_mining < bores_remaining && bores_completed < total_mining_runs || !gridBoreDataList[drone_gps_grid_list_position[i]].boreOccupied && !gridBoreDataList[drone_gps_grid_list_position[i]].boreFinished && !drone_mining[i])
                        {
                            drone_must_wait[i] = false;
                        }
                        if (bores_completed != total_mining_runs && !drone_must_wait[i])
                        {
                            drone_control_sequence[i] = 1;
                            drone_mining[i] = true;
                            GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                            bore.boreOccupied = true;
                            gridBoreDataList[drone_gps_grid_list_position[i]] = bore;
                        }
                        else
                        {
                            drone_control_sequence[i] = 0;
                            drone_mining[i] = false;
                        }
                        if (gridBoreDataList[drone_gps_grid_list_position[i]].boreFinished)
                        {
                            //suspect coordinates here 2
                            Echo($"Drone position finished {i}");
                            drone_control_sequence[i] = 0;
                            drone_mining[i] = false;
                            drone_assigned_coordinates[i] = false;
                            drone_gps_grid_list_position[i] = -1;
                        }
                    }
                }
                tx_chan = drone_name[i];
                currentDroneBoreIndex = gps_grid_position_value.ToString();
                commandOutput = "0";
                currentMiningDroneBoreLocationX = Math.Round(drone_gps_coordinates_ds[i].X, 2).ToString();
                currentMiningDroneBoreLocationY = Math.Round(drone_gps_coordinates_ds[i].Y, 2).ToString();
                currentMiningDroneBoreLocationZ = Math.Round(drone_gps_coordinates_ds[i].Z, 2).ToString();
                currentMiningDroneCustomData_5 = cst_dt5;
                currentMiningDroneBoreLength = (drillLength + safe_dstvl).ToString();
                currentMiningDroneIgnoreDistance = (ignoreDepth + safe_dstvl + drone_length).ToString();
                if (align_target_valid)
                {
                    currentMiningDroneAlignLocationX = Math.Round(((drone_gps_coordinates_ds[i].X - mainGPSCoordinates.X) + alignGPSCoordinates.X), 2).ToString();
                    currentMiningDroneAlignLocationY = Math.Round(((drone_gps_coordinates_ds[i].Y - mainGPSCoordinates.Y) + alignGPSCoordinates.Y), 2).ToString();
                    currentMiningDroneAlignLocationZ = Math.Round(((drone_gps_coordinates_ds[i].Z - mainGPSCoordinates.Z) + alignGPSCoordinates.Z), 2).ToString();
                }
                else
                {
                    currentMiningDroneAlignLocationX = "";
                    currentMiningDroneAlignLocationY = "";
                    currentMiningDroneAlignLocationZ = "";
                }
                if (drone_control_sequence[i] == 1 && drone_assigned_coordinates[i] && !drone_must_wait[i] && !run_arg || drone_control_sequence[i] == 2 && drone_control_status[i] == "Docked Idle" && drone_dock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 2;
                    drone_mining[i] = true;
                    GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                    bore.boreOccupied = true;
                    gridBoreDataList[drone_gps_grid_list_position[i]] = bore;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "7";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 2 && drone_control_status[i] == "Undocked" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg || drone_control_sequence[i] == 2 && drone_control_status[i] == "Docking" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 3;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (drone_control_sequence[i] == 2 && drone_control_status[i] == "Undocking" && drone_dock_status[i] == "False" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg && dcs[i] <= bclu)
                {
                    drone_control_sequence[i] = 13;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 13 && drone_control_status[i] == "Idle" && drone_dock_status[i] == "False" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg || drone_control_sequence[i] == 5 && drone_control_status[i] == "Docking" && drone_dock_status[i] == "False" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg && dcs[i] <= bclu)
                {
                    drone_control_sequence[i] = 8;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "6";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 3 && drone_control_status[i] == "Idle" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 4;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "4";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 4 && drone_control_status[i] == "Nav End" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 5;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 4 && drone_control_status[i] == "Docked Idle" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 1;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (drone_control_sequence[i] == 5 && drone_control_status[i] == "Idle" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 6;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "2";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (drone_control_sequence[i] == 6 && drone_control_status[i] == "Nav End" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 7;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 7 && drone_control_status[i] == "Idle" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 8;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "5";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] >= 8 && drone_control_status[i].Contains("Docked") && drone_mining[i] || drone_control_sequence[i] == 4 && drone_control_status[i].Contains("Docked") && drone_mining[i])
                {
                    GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                    bore.boreOccupied = false;
                    gridBoreDataList[drone_gps_grid_list_position[i]] = bore;
                }
                if (drone_control_sequence[i] >= 8 && drone_control_status[i].Contains("Dock") && drone_mining[i] && drone_tunnel_complete[i] == "True")
                {
                    GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                    bore.boreFinished = true;
                    gridBoreDataList[drone_gps_grid_list_position[i]] = bore;
                }
                if (drone_control_sequence[i] == 8 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && !run_arg)
                {
                    drone_control_sequence[i] = 1;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 8 && !drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 8 && !drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] >= 1 && drone_control_sequence[i] <= 4 && !drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && !run_arg)
                {
                    drone_control_sequence[i] = 0;
                    drone_assigned_coordinates[i] = false;
                    drone_mining[i] = false;
                    gps_grid_position_value = -1;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 8 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg)
                {
                    drone_control_sequence[i] = 9;
                    GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                    bore.boreFinished = true;
                    gridBoreDataList[drone_gps_grid_list_position[i]] = bore;

                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 9 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && can_run && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 9 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && drone_assigned_coordinates[i] && !run_arg)
                {
                    drone_control_sequence[i] = 10;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 10 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && general_reset && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 10 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 0 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg)
                {
                    drone_control_sequence[i] = 11;
                    drone_tunnel_complete[i] = "False";
                    GridBoreData bore = gridBoreDataList[drone_gps_grid_list_position[i]];
                    bore.boreFinished = true;
                    gridBoreDataList[drone_gps_grid_list_position[i]] = bore;
                    t_mne_sq_cmp++;
                    gps_grid_position_value = -1;
                    drone_reset_func[i] = false;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "8";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 11 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && t_mne_sq_cmp <= total_mining_runs && mining_grid_valid && !run_arg)
                {
                    drone_control_sequence[i] = 0;
                    drone_assigned_coordinates[i] = false;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_sequence[i] == 11 && drone_control_status[i].Contains("Docked") && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && current_gps_idx < total_mining_runs && drone_assigned_coordinates[i] && t_mne_sq_cmp > total_mining_runs && !run_arg || drone_control_sequence[i] == 11 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && mining_grid_valid == false && t_mne_sq_cmp >= total_mining_runs && !run_arg)
                {
                    drone_control_sequence[i] = 12;
                    drone_assigned_coordinates[i] = false;
                    gps_grid_position_value = -1;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }

                    dp_txm.Append('\n');
                    dp_txm.Append("Mining seq. complete");
                }
                if (drone_control_status[i].Contains("Docked") && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && general_reset || drone_control_status[i].Contains("Docked") && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && general_reset && !run_arg)
                {
                    drone_control_sequence[i] = 0;
                    t_mne_sq_cmp = 0;
                    drone_tunnel_complete[i] = "False";
                    drone_assigned_coordinates[i] = false;
                    drone_mining[i] = false;
                    current_gps_idx = 0;
                    gps_grid_position_value = -1;
                    drone_reset_func[i] = false;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "8";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }
                if (drone_control_status[i].Contains("Docked") && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && general_reset && drone_control_sequence[i] == 0 && !run_arg || drone_control_status[i].Contains("Docked") && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && general_reset && !run_arg || drone_control_sequence[i] == 6 && drone_control_status[i] == "Docked Idle" && drone_dock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                {
                    drone_control_sequence[i] = 0;
                    t_mne_sq_cmp = 0;
                    drone_tunnel_complete[i] = "False";
                    drone_assigned_coordinates[i] = false;
                    drone_mining[i] = false;
                    current_gps_idx = 0;
                    gps_grid_position_value = -1;
                    drone_reset_func[i] = false;
                    currentDroneBoreIndex = gps_grid_position_value.ToString();
                    commandOutput = "0";
                    drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                    droneTranmissionOutput[i] = c.ToString();
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (mustRecall_Command && !drone_recall_list[i] && !mustUndock_Command)
                {
                    drone_recall_list[i] = true;
                }
                if (drone_recall_list[i])
                {
                    if (drone_recall_sequence[i] == 0 && drone_control_status[i] == "Idle" || drone_recall_sequence[i] == 0 && drone_control_status[i] == "Undocked" || drone_recall_sequence[i] == 0 && drone_control_status[i] == "Nav" || drone_recall_sequence[i] == 0 && drone_control_status[i] == "Undocking" || drone_recall_sequence[i] == 0 && drone_control_status[i] == "Docking" || drone_recall_sequence[i] == 0 && drone_control_status[i] == "Initiating mining")
                    {
                        drone_recall_sequence[i] = 1;
                    }

                    if (drone_recall_sequence[i] == 0 && drone_control_status[i] == "Nav End")
                    {
                        drone_recall_sequence[i] = 3;
                    }
                    if (drone_recall_sequence[i] == 1)
                    {
                        drone_recall_sequence[i] = 2;
                        drone_control_sequence[i] = 0;
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "0";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();
                    }
                    if (drone_recall_sequence[i] == 2 && drone_control_status[i] == "Idle")
                    {
                        drone_recall_sequence[i] = 3;
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "1";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (drone_recall_sequence[i] == 3 && drone_control_status[i] == "Nav End")
                    {
                        drone_recall_sequence[i] = 4;
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "0";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (drone_recall_sequence[i] == 3 && drone_control_status[i] == "Nav" && drone_gps_grid_list_position[i] == -1 || drone_recall_sequence[i] == 3 && drone_control_status[i] == "Idle" && drone_gps_grid_list_position[i] >= -1)
                    {
                        drone_recall_sequence[i] = 4;
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "0";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();

                    }

                    if (drone_recall_sequence[i] == 4 && drone_control_status[i] == "Idle")
                    {
                        drone_recall_sequence[i] = 5;
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "6";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (drone_recall_sequence[i] == 4 && drone_control_status[i] == "Idle")
                    {
                        drone_recall_sequence[i] = 5;
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "6";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (drone_recall_sequence[i] == 5 && drone_control_status[i].Contains("Docked") || drone_recall_sequence[i] == 0 && drone_control_status[i].Contains("Docked"))
                    {
                        drone_recall_sequence[i] = 0;
                        drone_assigned_coordinates[i] = false;
                        drone_recall_list[i] = false;
                        drone_mining[i] = false;
                        gps_grid_position_value = -1;
                        drone_reset_func[i] = true;
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "0";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();

                    }
                    if (canTransmit && droneTransmissionStatus[i])
                    {
                        transmit_to_drone();
                        droneTransmissionStatus[i] = false;
                    }
                }

                if (mustUndock_Command)
                {
                    if (drone_control_status[i] == "Docked Idle")
                    {
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "7";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();
                        if (canTransmit && droneTransmissionStatus[i])
                        {
                            transmit_to_drone();
                            droneTransmissionStatus[i] = false;
                        }
                    }
                }

                if (mustFreeze_Command)
                {
                    if (drone_control_status[i] == "Undocked" || drone_control_status[i] == "Idle")
                    {
                        gps_grid_position_value = drone_gps_grid_list_position[i];
                        currentDroneBoreIndex = gps_grid_position_value.ToString();
                        commandOutput = "0";
                        drone_command_builder(currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
                        droneTranmissionOutput[i] = c.ToString();
                        if (canTransmit && droneTransmissionStatus[i])
                        {
                            transmit_to_drone();
                            droneTransmissionStatus[i] = false;
                        }
                    }
                }
                Confirmed_Drone_Message = false;
                recieved_drone_name_index = -1;


                if (drone_messages_list.Count > 0)
                {
                    drone_messages_list.RemoveAt(0);
                }


            }
            #endregion
        }

        private void ProcessRecallCommand()
        {
            if (mustRecall_Command)
            {
                IGC.SendBroadcastMessage(tx_recall_channel, command_recall, TransmissionDistance.TransmissionDistanceMax);
            }
            else
            {
                IGC.SendBroadcastMessage(tx_recall_channel, command_operate, TransmissionDistance.TransmissionDistanceMax);
            }
            if (mustRecall_Command && current_gps_idx > 0)
            {
                current_gps_idx = 0;
            }
        }

        private void timeCountReset()
        {
            time_delay = false;
            time_count = 0;
        }

        private void UpdateActiveDroneLimits()
        {
            #region active_drones_processing
            //drone limit processing
            if (drone_name.Count > 0)
            {
                max_active_drone_count = drone_name.Count - flight_factor;
                if (max_active_drone_count <= 1)
                {
                    max_active_drone_count = 1;
                }
                if (max_active_drone_count > hard_active_drone_limit)
                {
                    max_active_drone_count = hard_active_drone_limit;
                }
            }
            #endregion
        }

        public struct GridBoreData
        {
            public Vector3D borePosition;
            public bool boreOccupied;
            public bool boreFinished;
            public GridBoreData(Vector3D borePosition, bool boreOccupied, bool boreFinished)
            {
                this.borePosition = borePosition;
                this.boreOccupied = boreOccupied;
                this.boreFinished = boreFinished;
            }
        }
        private void ProcessJobGrid()
        {
            #region job_grid_processing
            //if mining grid data empty resolve issues to avoid exception
            if (nPtsY == 0 && !created_grid || nPtsX == 0 && !created_grid || gridSize == 0 && !created_grid)
            {
                gridBoreDataList = new List<GridBoreData>();
                created_grid = true;
                centralGridGPSCoordinates = mainGPSCoordinates;  
                gridBoreDataList.Add(new GridBoreData(centralGridGPSCoordinates, false, false));
                total_mining_runs = gridBoreDataList.Count;
                current_gps_idx = 0;
                if (readyFlag)
                {
                    readyFlag = false;
                }
            }
            if (!created_grid)
            {

                if (!bores_regen)
                {
                    gridBoreDataList = new List<GridBoreData>();
                    bores_regen = true;
                }

                if (readyFlag)
                {
                    readyFlag = false;
                }
                gridBoreDataList.Clear();
                Vector3D gravity = remote_control_actual.GetNaturalGravity();
                if (align_target_valid)
                {
                    planeNrml = ((mainGPSCoordinates - alignGPSCoordinates));
                }
                if (!align_target_valid)
                {
                    planeNrml = gravity;
                }

                planeNrml.Normalize();
                Vector3D perpendicularVector = Vector3D.CalculatePerpendicularVector(planeNrml);
                perpendicularVector.Normalize();
                Vector3D centerPoint = mainGPSCoordinates;
                //load from storage if present (test required)
                if (Storage != null && Storage != "" && !created_grid && bores_regen && !init_grid_complete)
                {
                    //added from init
                    current_gps_idx = 0;
                    r_gps_idx = current_gps_idx;
                    GetStoredData();
                    Echo("Grid positions restored");
                    can_loading = true;
                    Storage = null;
                    //reset everything else
                    reset_drone_data();
                    reset_drone_list();
                    pinged = false;
                    pngt_count = 0;
                    init_grid_complete = true;
                }
                //coroutine management grid creation
                if (gridCoroutine == null && !init_grid_complete && bores_regen || gridCoroutine != null && !gridCoroutine.MoveNext() && !init_grid_complete && bores_regen)
                {
                    gridCoroutine = GenGrdPosits(centerPoint, planeNrml, gridSize, nPtsX, nPtsY, core_out);
                }
                if (gridCoroutine != null && !init_grid_complete && bores_regen)
                {
                    // Check the current yield value
                    bool currentYield = gridCoroutine.Current;

                    // If the coroutine is finished, you can perform completion logic
                    if (!gridCoroutine.MoveNext())
                    {
                        // The coroutine has finished executing
                        Echo("Grid generation complete.");
                        gridCoroutine = null; // Reset the coroutine
                        GenGrdPosits(centerPoint, planeNrml, gridSize, nPtsX, nPtsY, core_out).Dispose();
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
                            init_grid_complete = true;
                            pb_i_act.CustomData = "";
                            can_init = false;
                            i_init = false;
                            it_ag = "";
                        }
                    }

                }
                //grid data found - terminite initialisation
                if (gridBoreDataList.Count > 0 && init_grid_complete)
                {
                    created_grid = true;
                    pb_i_act.CustomData = "";
                    can_init = false;
                    i_init = false;
                    it_ag = "";
                }

                total_mining_runs = gridBoreDataList.Count;

                if (nPtsY == 0 || nPtsY == 0 || gridSize == 0 || nPtsY == 0 && nPtsY == 0 && gridSize == 0)
                {
                    mining_grid_valid = false;
                }
                if (nPtsY > 0 && nPtsY > 0 && gridSize > 0)
                {
                    mining_grid_valid = true;
                }
                if (!mining_grid_valid)
                {
                    total_mining_runs = 1;
                }
                t_mne_sq_cmp = 0;
                bores_completed = 0;
                current_gps_idx = 0;
            }
            Echo($"Grid: {created_grid} - Bores: {total_mining_runs} - Remaining: {bores_remaining}");
            #endregion
        }

        private void ProcessMessages()
        {
            #region check_drone_messages
            //manage recieved communications
            if (ant_act != null)
            {
                if (listen.HasPendingMessage)
                {
                    MyIGCMessage new_drone_message = listen.AcceptMessage();
                    drone_messages_list.Add(new_drone_message);
                }

                //process drone message list here
                if (drone_messages_list.Count < drone_name.Count)
                {
                    Drone_Message = true;
                }


                if (drone_messages_list.Count > 0)
                {
                    //pull first message in the list if valid
                    dataMessageFromDrone = drone_messages_list[0].Data.ToString();
                    Get_Drone_Message_Data(dataMessageFromDrone);
                    //recieved_drone_message_to_database();
                }

                if (drone_messages_list.Count > drone_name.Count)
                {
                    Drone_Message = false;
                }


                #endregion

                #region check_prospector_messages
                //process drone message list here
                if (listen_prspt.HasPendingMessage)
                {
                    MyIGCMessage new_prospector_message = listen_prspt.AcceptMessage();
                    prospector_messages_list.Add(new_prospector_message);

                }
                //process prospector message list here
                if (prospector_messages_list.Count <= 0)
                {
                    Prospect_Message = false;
                }
                if (prospector_messages_list.Count > 0)
                {
                    Prospect_Message = true;
                    //move this to prospect message management
                    data_in_prospector = prospector_messages_list[0].Data.ToString();
                    remote_control_actual.CustomData = data_in_prospector;
                    prospector_messages_list.RemoveAt(0);
                    created_grid = false;
                }
            }
            #endregion
        }

        private void PingDrones()
        {
            #region drone_lifecheck_ping
            //manage drone ping communications
            if (ant_act != null)
            {

                if (drone_name.Count == 0 && !pinged || drone_name.Count > 0 && !pinged)
                {
                    IGC.SendBroadcastMessage(tx_ping_channel, p_cht, TransmissionDistance.TransmissionDistanceMax);
                    pinged = true;
                    pngt_count = 0;
                }
            }
            #endregion
        }

        private void ValidateCustomData()
        {
            #region check_custom_data_validation
            if (Me.CustomData != null && Me.CustomData != "")
            {
                dat_vld = true;
            }
            else
            {
                dat_vld = false;

                miningCoordinatesNew.Clear();
                miningCoordinatesNew.Append("GPS" + ":" + "---" + ":" + 0 + ":" + 0 + ":" + 0 + ":" + "#FF75C9F1" + ":" + "5.0" + ":" + "10.0" + ":" + "1" + ":" + "1" + ":" + "0" + ":" + "False" + ":" + "1" + ":" + "10" + ":" + "0" + ":");
                Me.CustomData = miningCoordinatesNew.ToString();
            }
            if (dat_vld)
            {
                GetCustomData_JobCommand();
            }
            #endregion
        }

        private void InitializeMiningGrid()
        {
            #region initialise_mining_grid_loading
            if (!init_grid_complete && initgridcount > 0)
            {
                initgridcount = 0;
            }
            if (init_grid_complete && initgridcount >= 1 || can_init && init_grid_complete)
            {
                initgridcount = 0;
                init_grid_complete = false;
            }
            if (can_init && created_grid && !init_grid_complete && !can_loading)
            {
                created_grid = false;
                mining_grid_valid = false;
                current_gps_idx = 0;
                gps_grid_position_value = -1;
                bores_regen = false;
                init_grid_complete = false;
            }
            if (can_loading)
            {
                can_loading = false;
            }
            #endregion
        }

        private void CheckSystemStatus()
        {
            if (!setup_complete)
            {
                setup_system();
                setup_complete = true;
                Echo("Setup complete!");
            }

            presence_check();
        }

        private void ProcessInterface()
        {
            #region Interface_detection
            if (pb_tg.Count > 0)
            {
                pb_i_act = pb_tg[0];
                can_intf = true;
                it_ag = pb_i_act.CustomData;
                Echo($"Interface PB: {intfc_tag}");
                Echo($"Display command: {it_ag}");
            }
            #endregion
            #region interface_command_processing
            if (can_intf && pb_i_act.CustomData != null)
            {
                if (it_ag == "" && !n_intf)
                {
                    n_intf = true;
                }
                else
                {
                    n_intf = false;
                }
                if (it_ag.Contains("init") && !i_init)
                {
                    i_init = true;
                }
                if (!it_ag.Contains("init") && i_init)
                {
                    i_init = false;
                }
                if (it_ag.Contains("reset") && !i_res)
                {
                    i_res = true;
                }
                else
                {
                    i_res = false;
                }
                if (it_ag.Contains("run") && !i_run)
                {
                    i_run = true;
                }
                else
                {
                    i_run = false;
                }
                if (it_ag.Contains("recall") && !i_recall)
                {
                    i_recall = true;
                }
                else
                {
                    i_recall = false;
                }
                if (it_ag.Contains("eject") && !i_eject)
                {
                    i_eject = true;
                }
                else
                {
                    i_eject = false;
                }
                if (it_ag.Contains("freeze") && !i_frz)
                {
                    i_frz = true;
                }
                else
                {
                    i_frz = false;
                }
                if (it_ag.Contains("stop") && !i_stop)
                {
                    i_stop = true;
                }
                else
                {
                    i_stop = false;
                }
            }
            if (!can_intf || n_intf || pb_i_act.CustomData == null)
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
            if (argument == "setup" && setup_complete)
            {
                setup_complete = false;
                argument = "";
                Echo("Running Setup..");
            }
            if (argument.Contains("run") || i_run)
            {
                can_run = true;
                canReset = false;
                canTransmit = true;
                mustRecall_Command = false;
                can_init = false;
                mustFreeze_Command = false;
                commandAsk = "Run";
            }
            if (argument.Contains("reset") || i_res)
            {
                canReset = true;
                mustUndock_Command = false;
                can_run = false;
                canTransmit = true;
                mustRecall_Command = false;
                can_init = false;
                mustFreeze_Command = false;
                commandAsk = "Reset";
                current_gps_idx = 0;
            }
            if (argument.Contains("stop") || i_stop)
            {
                canTransmit = false;
                mustUndock_Command = false;
                can_run = false;
                canReset = false;
                mustRecall_Command = false;
                can_init = false;
                mustFreeze_Command = false;
                commandAsk = "Stop";
            }
            if (argument.Contains("recall") || i_recall)
            {
                mustRecall_Command = true;
                mustUndock_Command = false;
                canReset = false;
                canTransmit = true;
                can_run = false;
                mustFreeze_Command = false;
                commandAsk = "Recall";
                current_gps_idx = 0;
            }
            if (argument.Contains("init") || i_init)
            {
                mustRecall_Command = false;
                mustUndock_Command = false;
                canReset = false;
                canTransmit = false;
                can_run = false;
                can_init = true;
                mustFreeze_Command = false;
                commandAsk = "Init";
                current_gps_idx = 0;
                r_gps_idx = current_gps_idx;
                Storage = null;

            }
            if (argument.Contains("eject") || i_eject)
            {
                mustRecall_Command = false;
                mustUndock_Command = true;
                canReset = false;
                canTransmit = true;
                can_run = false;
                mustFreeze_Command = false;
                commandAsk = "Eject";
                current_gps_idx = 0;
            }
            if (argument.Contains("freeze") || i_frz)
            {
                canTransmit = true;
                mustFreeze_Command = true;
                mustUndock_Command = false;
                can_run = false;
                canReset = false;
                mustRecall_Command = false;
                can_init = false;
                commandAsk = "Freeze";
            }
            if (mustUndock_Command || mustRecall_Command || mustFreeze_Command)
            {
                run_arg = true;
            }
            else
            {
                run_arg = false;
            }
            #endregion
        }

        private void drone_render_call()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (display_tag_drone.Count == 0 || drone_name.Count == 0 || display_tag_drone[0] == null) return;

            if (renew_header)
            {
                droneInformation.Clear().Append($"Mining Drone Status - GMDC {ver}\n");
                renew_header = false;
            }

            int dronesPerDisplay = drones_per_screen * display_tag_drone.Count;
            if (dronesPerDisplay < drone_name.Count)
            {
                Echo($"Insufficient displays '{dp_drn_tag}': {dronesPerDisplay} < {drone_name.Count}");
                return;
            }

            for (int i = 0; i < drone_name.Count; i += 2)
            {
                bool hasPair = i + 1 < drone_name.Count;
                scrnbldr(i, hasPair ? i + 1 : i, hasPair);

                int displayIndex = i / drones_per_screen;
                if (displayIndex < display_tag_drone.Count && display_tag_drone[displayIndex] != null &&
                    (i % drones_per_screen == drones_per_screen - 2 || i >= drone_name.Count - 2))
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
            bores_completed = CntTrueVlsGridBoreDataFinished(gridBoreDataList);
            bores_remaining = total_mining_runs - bores_completed;
            total_drones_mining = CntTrueVls(drone_mining);
            total_drones_undocking = CntIntVls(drone_control_sequence, 2);

            DroneStats stats = new DroneStats();
            for (int i = 0; i < drone_name.Count; i++)
            {
                string status = drone_control_status[i];
                string damage = drone_damage_state[i];
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
            t_drn_dmg = stats.Damage; t_dn_unk = stats.Unknown; t_dn_ok = stats.Ok;

            Echo($"UpdateDroneCounts: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private int[] boreQueueCounts = new int[0]; // Static reuse

        private void DroneUndockCheck()
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            if (total_drones_undocking > 0)
                drones_undocking = true;
            if (drones_undocking)
            {
                undock_timer++;
                if (undock_timer >= undock_delay_limit)
                {
                    drones_undocking = false;
                    undock_timer = 0;
                }
            }
            else if (undock_timer != 0)
                undock_timer = 0;

            if (gridBoreDataList.Count > 0)
            {
                if (boreQueueCounts.Length < gridBoreDataList.Count)
                    boreQueueCounts = new int[gridBoreDataList.Count];
                Array.Clear(boreQueueCounts, 0, gridBoreDataList.Count);

                for (int d = 0; d < drone_gps_grid_list_position.Count; d++)
                    if (drone_gps_grid_list_position[d] >= 0 && drone_gps_grid_list_position[d] < boreQueueCounts.Length)
                        boreQueueCounts[drone_gps_grid_list_position[d]]++;

                for (int l = 0; l < gridBoreDataList.Count; l++)
                {
                    GridBoreData bore = gridBoreDataList[l];
                    if (gridBoreDataList[l].boreOccupied && boreQueueCounts[l] == 0)
                    {
                        bore.boreOccupied = false;
                        gridBoreDataList[l] = bore;
                    }                        
                }
            }
            Echo($"DroneUndockCheck: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        public void updateDisplay(int i)
        {
            dp_txm.Append($"Drone Controller Status - GMDC {ver} - [{drone_tag}] {icon}");
            dp_txm.Append('\n');
            if (drone_control_sequence[i] == 12)
            {
                gps_grid_position_value = -1;
                drone_mining[i] = false;
                dp_txm.Append('\n');
                dp_txm.Append("Mining seq. complete"); ;
            }

            if (gridBoreDataList.Count > 0)
            {
                dp_txm.Append('\n');
                dp_txm.Append("Grid pos: " + gridBoreDataList.Count);
                dp_txm.Append('\n');
                dp_txm.Append("Grid dist: " + cst_dt7 + "m #X: " + cst_dt8 + " #Y: " + cst_dt9);
                dp_txm.Append('\n');
                dp_txm.Append("Grid OK: " + mining_grid_valid);
                dp_txm.Append('\n');
                dp_txm.Append("Bores: " + gridBoreDataList.Count + " Remain: " + bores_remaining + "  Skip: " + skip_bores_number);
                dp_txm.Append('\n');
            }
            else
            {
                dp_txm.Append('\n');
                dp_txm.Append("Bores: " + total_mining_runs + " Remaining: " + bores_remaining);
            }
            if (total_mining_runs > 0)
            {
                dp_txm.Append('\n');
                dp_txm.Append("Current mine idx: " + r_gps_idx + " of " + (total_mining_runs - 1) + " (" + bores_completed + ") ");
            }
            else
            {
                dp_txm.Append('\n');
                dp_txm.Append("Current mine idx: " + current_gps_idx + " of " + (total_mining_runs - 1) + " (" + bores_completed + ") ");
            }
            if (current_gps_idx > total_mining_runs || !mining_grid_valid && bores_completed >= total_mining_runs || bores_remaining == 0)
            {
                can_run = false;
                dp_txm.Append('\n');
                dp_txm.Append("Mine seq. complete");
                dp_txm.Append('\n');
                drone_control_sequence[i] = 12;
                current_gps_idx = 0;
            }
        }
        void Get_Drone_Message_Data(string data_message)
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            // get custom data from programmable block
            String[] messageData = data_message.Split(':');
            if (messageData.Length < 20 || !messageData[0].Contains(drone_tag))
            {
                Confirmed_Drone_Message = false;
                return;
            }

            int i = droneDataList.FindIndex(d=> d.droneName == messageData[0]);
            DroneData drone = new DroneData();
            
            if (i >= 0)
            {
                drone = droneDataList[i];                
            }
            else
            {
                drone = new DroneData(messageData[0], "OK", false, "Idle", false, false, false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1, false, false, false);  
            }
            //drone.droneName = messageData[0]; - not needed as it gets picked up from message
            //populate drone data from message data
            drone.droneDamage = messageData[1];
            if (!bool.TryParse(messageData[2], out drone.droneTunnelFinished))
            {
                drone.droneTunnelFinished = false;
            }
            drone.droneStatusOutput = messageData[3];
            if (!bool.TryParse(messageData[4], out drone.isDocked))
            {
                drone.isDocked = false;
            }
            if (!bool.TryParse(messageData[5], out drone.isUndocked))
            {
                drone.isUndocked = false;
            }
            if (!bool.TryParse(messageData[6], out drone.isAutopiloting))
            {
                drone.isAutopiloting = false;
            }
            if (!bool.TryParse(messageData[7], out drone.rcAutopilotEnabled))
            {
                drone.rcAutopilotEnabled = false;
            }
            if (!double.TryParse(messageData[8], out drone.droneLocationX))
            {
                drone.droneLocationX = 0.0;
            }
            if (!double.TryParse(messageData[9], out drone.droneLocationY))
            {
                drone.droneLocationY = 0.0;
            }
            if (!double.TryParse(messageData[10], out drone.droneLocationZ))
            {
                drone.droneLocationZ = 0.0;
            }
            if (!double.TryParse(messageData[11], out drone.currentBoreLength))
            {
                drone.currentBoreLength = 0.0;
            }
            if (!double.TryParse(messageData[12], out drone.currentBoreDistance))
            {
                drone.currentBoreDistance = 0.0;
            }
            if (!double.TryParse(messageData[13], out drone.currentBoreMineDistance))
            {
                drone.currentBoreMineDistance = 0.0;
            }
            if (!double.TryParse(messageData[14], out drone.currentDroneCharge))
            {
                drone.currentDroneCharge = 0.0;
            }
            if (!double.TryParse(messageData[15], out drone.currentDroneGas))
            {
                drone.currentDroneGas = 0.0;
            }
            if (!double.TryParse(messageData[16], out drone.currentDroneOre))
            {
                drone.currentDroneOre = 0.0;
            }
            if (!int.TryParse(messageData[17], out drone.currentGPSIndex))
            {
                drone.currentGPSIndex = -1;
            }
            if (!bool.TryParse(messageData[18], out drone.cargoFull))
            {
                drone.cargoFull = false;
            }
            if (!bool.TryParse(messageData[19], out drone.rechargeRequest))
            {
                drone.rechargeRequest = false;
            }
            if (!bool.TryParse(messageData[20], out drone.autoDock))
            {
                drone.autoDock = false;
            }
            if (i >= 0)
            {                
                droneDataList[i] = drone; // if drone already exists update it
                Confirmed_Drone_Message = true;
                recieved_drone_name_index = i;
            }
            else
            {                              
                droneDataList.Add(drone); // if drone does not exist add it
                Confirmed_Drone_Message = true;
                recieved_drone_name_index = droneDataList.Count - 1;
            }
        }

        void GetRemoteControlData()
        {
            String[] remoteGpsCommand = remote_control_actual.CustomData.Split(':');

            if (remoteGpsCommand.Length < 6)
            {
                rm_cst_dat1 = "";
                rm_cst_dat2 = "";
                rm_cst_dat3 = "";
                rm_cst_dat4 = "";
                rm_cst_dat5 = "";
                rm_cst_dat6 = "";
                target_valid = true;
                return;
            }
            if (remoteGpsCommand.Length > 6)
            {
                //target_gps_coords = new Vector3D(Double.Parse(remoteGpsCommand[2]), Double.Parse(remoteGpsCommand[3]), Double.Parse(remoteGpsCommand[4]));
                target_valid = true;
                rm_cst_dat1 = remoteGpsCommand[1];
                rm_cst_dat2 = remoteGpsCommand[2];
                rm_cst_dat3 = remoteGpsCommand[3];
                rm_cst_dat4 = remoteGpsCommand[4];
                rm_cst_dat5 = remoteGpsCommand[5];
                rm_cst_dat6 = remoteGpsCommand[6];
                if (!double.TryParse(rm_cst_dat2, out target_gps_coords.X))
                {
                    target_gps_coords.X = 0.0;
                    rm_cst_dat2 = "";
                }
                if (!double.TryParse(rm_cst_dat3, out target_gps_coords.Y))
                {
                    target_gps_coords.Y = 0.0;
                    rm_cst_dat3 = "";
                }
                if (!double.TryParse(rm_cst_dat4, out target_gps_coords.Z))
                {
                    target_gps_coords.Z = 0.0;
                    rm_cst_dat4 = "";
                }
                //5 is colour data
                if (!double.TryParse(rm_cst_dat6, out safe_dstvl))
                {
                    safe_dstvl = 0.0;
                }

            }
            if (remoteGpsCommand.Length < 12 && remoteGpsCommand.Length > 7)
            {
                rm_cst_dat7 = "";
                rm_cst_dat8 = "";
                rm_cst_dat9 = "";
                rm_cst_dat10 = "";
                rm_cst_dat11 = "";
                rm_cst_dat12 = "";
                align_target_valid = false;
                return;
            }
            if (remoteGpsCommand.Length > 7)
            {
                align_target_valid = true;
                rm_cst_dat7 = remoteGpsCommand[7];
                rm_cst_dat8 = remoteGpsCommand[8];
                rm_cst_dat9 = remoteGpsCommand[9];
                rm_cst_dat10 = remoteGpsCommand[10];
                rm_cst_dat11 = remoteGpsCommand[11];
                rm_cst_dat12 = remoteGpsCommand[12];
                if (!double.TryParse(rm_cst_dat9, out alignGPSCoordinates.X))
                {
                    alignGPSCoordinates.X = 0.0;
                    rm_cst_dat9 = "";
                }
                if (!double.TryParse(rm_cst_dat10, out alignGPSCoordinates.Y))
                {
                    alignGPSCoordinates.Y = 0.0;
                    rm_cst_dat10 = "";
                }
                if (!double.TryParse(rm_cst_dat11, out alignGPSCoordinates.Z))
                {
                    alignGPSCoordinates.Z = 0.0;
                    rm_cst_dat11 = "";
                }
            }
        }

        void GetCustomData_JobCommand()
        {
            String[] gps_cmnd = Me.CustomData.Split(':');
            if (gps_cmnd.Length < 10)
            {
                cst_dt1 = "";
                cst_dt2 = "";
                cst_dt3 = "";
                cst_dt4 = "";
                cst_dt5 = "";
                cst_dt6 = "";
                cst_dt7 = "";
                cst_dt8 = "";
                cst_dt9 = "";
                cst_dt10 = "";
                cst_dt11 = "";
                cst_dt12 = "";
                cst_dt13 = "";
                cst_dt14 = "";
                cst_dt15 = "";

                Echo("Format invalid,GPS:name:x:y:z:depth:grid:numx:numy:limit=True/False:flightfactor:flighthardlimit:skipboresnum");
                Echo("Set grid, numx, numy to 1 for single point");
                return;
            }


            if (gps_cmnd.Length > 4)
            {
                cst_dt1 = gps_cmnd[1];
                cst_dt2 = gps_cmnd[2];
                cst_dt3 = gps_cmnd[3];
                cst_dt4 = gps_cmnd[4];
                cst_dt5 = gps_cmnd[5];
                if (!double.TryParse(cst_dt2, out mainGPSCoordinates.X))
                {
                    mainGPSCoordinates.X = 0.0;
                    cst_dt2 = "";
                }
                if (!double.TryParse(cst_dt3, out mainGPSCoordinates.Y))
                {
                    mainGPSCoordinates.Y = 0.0;
                    cst_dt3 = "";
                }
                if (!double.TryParse(cst_dt4, out mainGPSCoordinates.Z))
                {
                    mainGPSCoordinates.Z = 0.0;
                    cst_dt4 = "";
                }
            }
            //5 should be colour data
            if (gps_cmnd.Length > 5)
            {
                cst_dt6 = gps_cmnd[6];
                if (!Double.TryParse(cst_dt6, out drillLength))
                {
                    drillLength = 1.0;
                    cst_dt6 = "";
                }
            }
            if (gps_cmnd.Length > 6)
            {
                cst_dt7 = gps_cmnd[7];
                if (!Double.TryParse(cst_dt7, out gridSize))
                {
                    gridSize = 0.0;
                    cst_dt7 = "";
                }
            }
            if (gps_cmnd.Length > 7)
            {
                cst_dt8 = gps_cmnd[8];
                if (!int.TryParse(cst_dt8, out nPtsX))
                {
                    nPtsX = 0;
                    cst_dt8 = "";
                }
            }
            if (gps_cmnd.Length > 8)
            {
                cst_dt9 = gps_cmnd[9];

                if (!int.TryParse(cst_dt9, out nPtsY))
                {
                    nPtsY = 0;
                    cst_dt9 = "";
                }
            }
            if (gps_cmnd.Length > 9)
            {
                cst_dt10 = gps_cmnd[10];
                if (!Double.TryParse(cst_dt10, out ignoreDepth))
                {
                    ignoreDepth = 0.0;
                    cst_dt10 = "";
                }
            }
            if (gps_cmnd.Length > 10)
            {
                cst_dt11 = gps_cmnd[11];
                if (!bool.TryParse(cst_dt11, out launched_drone_status))
                {
                    launched_drone_status = false;
                    cst_dt11 = "";
                }
            }
            if (gps_cmnd.Length > 11)
            {
                cst_dt12 = gps_cmnd[12];
                if (!int.TryParse(cst_dt12, out flight_factor))
                {
                    flight_factor = 1;
                    cst_dt12 = "";
                }
            }
            if (gps_cmnd.Length > 12)
            {
                cst_dt13 = gps_cmnd[13];
                if (!int.TryParse(cst_dt13, out hard_active_drone_limit))
                {
                    hard_active_drone_limit = 6;
                    cst_dt13 = "";
                }
            }
            if (gps_cmnd.Length > 13)
            {
                cst_dt14 = gps_cmnd[14];
                if (!int.TryParse(cst_dt14, out skip_bores_number))
                {
                    skip_bores_number = 0;
                    cst_dt14 = "";
                }
            }
            if (gps_cmnd.Length > 14)
            {
                cst_dt15 = gps_cmnd[15];
                if (!bool.TryParse(cst_dt15, out core_out))
                {
                    core_out = false;
                    cst_dt15 = "";
                }
            }

        }

        public void scrnbldr(int ivl, int ivl2, bool slu)
        {
            // Pre-build strings into cl/cl2 for padding
            cl[0] = $"{drone_name[ivl]} Status: {drone_damage_state[ivl]} {drone_control_status[ivl]}";
            cl[1] = $"{drone_name[ivl]} Docked: {drone_dock_status[ivl]}";
            cl[2] = $"{drone_name[ivl]} Undocked: {drone_undock_status[ivl]}";
            cl[3] = $"{drone_name[ivl]} Finished: {drone_tunnel_complete[ivl]}";
            cl[4] = $"{drone_name[ivl]} Mining: {drone_mining[ivl]}";
            cl[5] = $"{drone_name[ivl]} Waiting: {drone_must_wait[ivl]} Reset: {drone_reset_func[ivl]}";
            cl[6] = $"Charge: {drone_charge_storage[ivl]}% Tank: {drone_gas_storage[ivl]}% Cargo: {drone_ore_storage[ivl]}%";
            cl[7] = $"Drill depth: {drone_drill_depth_value[ivl]}m Start: {drone_mine_depth_start_status[ivl]}m";
            cl[8] = $"Current depth: {drone_mine_distance_status[ivl]}m";
            cl[9] = $"Drone control seq: {drone_control_sequence[ivl]} Recall seq: {drone_recall_sequence[ivl]} {drone_recall_list[ivl]}";
            cl[10] = $"Location: {drone_gps_grid_list_position[ivl]} Asnd: {drone_assigned_coordinates[ivl]} Unit OK: {droneStatusOK[ivl]}";
            cl[11] = $"X: {drone_location_x[ivl]} Y: {drone_location_y[ivl]} Z: {drone_location_z[ivl]}";

            if (slu)
            {
                cl2[0] = $"{drone_name[ivl2]} Status: {drone_damage_state[ivl2]} {drone_control_status[ivl2]}";
                cl2[1] = $"{drone_name[ivl2]} Docked: {drone_dock_status[ivl2]}";
                cl2[2] = $"{drone_name[ivl2]} Undocked: {drone_undock_status[ivl2]}";
                cl2[3] = $"{drone_name[ivl2]} Finished: {drone_tunnel_complete[ivl2]}";
                cl2[4] = $"{drone_name[ivl2]} Mining: {drone_mining[ivl2]}";
                cl2[5] = $"{drone_name[ivl2]} Waiting: {drone_must_wait[ivl2]} Reset: {drone_reset_func[ivl2]}";
                cl2[6] = $"Charge: {drone_charge_storage[ivl2]}% Tank: {drone_gas_storage[ivl2]}% Cargo: {drone_ore_storage[ivl2]}%";
                cl2[7] = $"Drill depth: {drone_drill_depth_value[ivl2]}m Start: {drone_mine_depth_start_status[ivl2]}m";
                cl2[8] = $"Current depth: {drone_mine_distance_status[ivl2]}m";
                cl2[9] = $"Drone control seq: {drone_control_sequence[ivl2]} Recall seq: {drone_recall_sequence[ivl2]} {drone_recall_list[ivl2]}";
                cl2[10] = $"Location: {drone_gps_grid_list_position[ivl2]} Asnd: {drone_assigned_coordinates[ivl2]} Unit OK: {droneStatusOK[ivl2]}";
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
            if (!listheader_generated)
            {
                dp_txl.Append("Mining Grid Status" + " - GMDC " + ver);
                dp_txl.Append('\n');
                dp_txl.Append('\n');
                dp_txl.Append("Remaining bores: " + bores_remaining);
                dp_txl.Append('\n');
                listheader_generated = true;
            }

            for (int i = 0; i < gridBoreDataList.Count; i++)
            {
                for (int j = 0; j < drone_gps_grid_list_position.Count; j++)
                {
                    if (!gridBoreDataList[i].boreOccupied)
                    {
                        drone_namer = "";
                    }
                    else if (i == drone_gps_grid_list_position[j])
                    {
                        drone_namer = drone_name[j];
                        drone_assigns_count[j]++;
                    }
                    if (drone_assigns_count[j] > 1)
                    {
                        GridBoreData bore = gridBoreDataList[i];
                        bore.boreOccupied = false;
                        gridBoreDataList[i] = bore;
                    }
                    drone_assigns_count[j] = 0;

                }
                if (!gridBoreDataList[i].boreFinished)
                {
                    dp_txl.Append('\n');
                    dp_txl.Append($"i: {i}  Mining: {gridBoreDataList[i].boreOccupied.ToString()}  Finished: {gridBoreDataList[i].boreFinished.ToString()} Drone: {drone_namer}");

                }
                if (i == gridBoreDataList.Count - 1)
                {
                    listgenerator_finished = true;
                }
                percent_list = ((double)i / (double)gridBoreDataList.Count) * 100;
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
                    gridBoreDataList.Add(new GridBoreData(position,false,false));
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
                            gridBoreDataList.Add(new GridBoreData(position, false, false));
                        }
                        yield return false;
                    }
                }
            }
            gridcount = gridcount_inner + gridcount_outer;
            percent_grid = (double)gridBoreDataList.Count / (double)gridcount;
            if (gridBoreDataList.Count == gridcount)
            {
                init_grid_complete = true;
            }
            else
            {
                init_grid_complete = false;
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

            if (gridBoreDataList.Count > 0)
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
                    Data = $"Total Bores: {total_mining_runs} - Remaining:{bores_remaining} - Drones: {total_drones_mining}",
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

                for (int i = 0; i < gridBoreDataList.Count; i++)
                {
                    sprite_total++;
                    Vector3D relativePoint = gridBoreDataList[i].borePosition - centerPoint;
                    double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                    double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                    var CentX = (float)xPlanar;
                    var CentY = (float)yPlanar;
                    string Image;
                    var bore_colour = new Color();
                    var alpha_bytes = 1.0f;
                    Image = gridBoreDataList[i].boreFinished ? "CircleHollow" : "Circle";
                    alpha_bytes = gridBoreDataList[i].boreOccupied ? 1.0f : 0.5f;
                    bore_colour = gridBoreDataList[i].boreOccupied ? Color.LightSkyBlue : Color.DeepSkyBlue;

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
                    percent_list_vis = (((double)i + (double)1) / ((double)gridBoreDataList.Count)) * 100;
                    spritecount++;
                    yield return false;
                }

                if (drone_name.Count > 0)
                {
                    drone_total = 0;
                    for (int i = 0; i < drone_name.Count; i++)
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

                        if (drone_control_status[i].Contains("Docked") || drone_control_status[i].Contains("Undocked") || drone_control_status[i].Contains("Docking") || drone_control_status[i].Contains("Undocking"))
                        {
                            alpha_val = 0.25f;
                        }
                        else
                        {
                            alpha_val = 1.0f;
                        }
                        if (!drone_mining[i])
                        {
                            Image_drone = "Circle";
                            bore_colour_drone = Color.Gray;
                        }
                        if (drone_mining[i])
                        {
                            Image_drone = "Circle";

                            if (drone_control_status[i].Contains("Min"))
                            {
                                bore_colour_drone = Color.Purple;
                            }
                            else if (drone_control_status[i].Contains("Exit"))
                            {
                                bore_colour_drone = Color.Orange;
                            }
                            else if (drone_control_status[i].Contains("RTB: Ready"))
                            {
                                bore_colour_drone = Color.Green;
                            }
                            else if (drone_control_status[i].Contains("Undock"))
                            {
                                bore_colour_drone = Color.Yellow;
                            }
                            else
                            {
                                bore_colour_drone = Color.Navy;
                            }
                            alpha_val = 1.0f;
                        }
                        if (drone_damage_state[i] == "DMG")
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
                            spritecount++;
                        }

                        if (drone_control_status[i].Contains("Recharg") || drone_control_status[i].Contains("Unload") || drone_recharge_request[i].Contains("True"))
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
                            spritecount++;
                        }

                        var position_text = new Vector2(CentX * scale_factor_x, CentY * scale_factor_y) + _viewport.Center;
                        //background sprite
                        bore_colour_drone = Color.WhiteSmoke;
                        var sprite_name = new MySprite()
                        {
                            Type = SpriteType.TEXT,
                            Data = $"{drone_name[i]}- ({drone_charge_storage[i]}%)",
                            Position = position,
                            RotationOrScale = 0.3f,
                            Size = sizer * 0.5f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER,
                            FontId = "White"
                        };
                        sprites.Add(sprite_name);
                        percent_list_drones = ((double)drone_total / (double)drone_name.Count) * 100;
                        spritecount++;
                        yield return false;
                    }
                }
                if (drone_name.Count == 0)
                {
                    if (sprite_total == gridBoreDataList.Count)
                    {
                        frame_generator_finished = true;
                    }

                    else
                    {
                        frame_generator_finished = false;
                    }
                }
                if (drone_name.Count > 0)
                {
                    if (sprite_total == gridBoreDataList.Count && drone_total == drone_name.Count)
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
            if (spritecount >= spritecount_limit_insert && !sprite_insert)
            {
                var banger = new MySprite();
                frame.Add(banger);
                Echo("Frame shift");
                sprite_insert = true;
                spritecount++;
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
            spritecount++;

            for (int i = 0; i < sprites.Count; i++)
            {
                frame.Add(sprites[i]);
            }
        }
        public void CyclNextCord()
        {
            current_gps_idx = (current_gps_idx + 1) % gridBoreDataList.Count;
            next_gps_crds = gridBoreDataList[current_gps_idx].borePosition;
        }


        #region drone_status_verification
        int CntTrueVls(List<bool> list)
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
        int CntIntVls(List<int> list, int val)
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
        int CntStsVls(List<string> list, string textval)
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


        public void drone_command_builder (
            string currentDroneBoreIndex, string currentMiningDroneBoreLocationX, string currentMiningDroneBoreLocationY, string currentMiningDroneBoreLocationZ, 
            string currentMiningDroneCustomData_5, string commandOutput, string currentMiningDroneBoreLength, string currentMiningDroneIgnoreDistance,                         
            string currentMiningDroneAlignLocationX, string currentMiningDroneAlignLocationY, string currentMiningDroneAlignLocationZ            
            )
        {
            //drone_command_builder(currentMiningDroneIgnoreDistance, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
            const string baseFormat = "GPS:{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:";
            const string astFormat = "GPS:PAD:{0}:{1}:{2}:#FF75C9F1:";
            /* Custom data message structure
                * 0  -   = GPS Text
                * 1 {0}  = GPS Index
                * 2 {1}  = Main Target X
                * 3 {2}  = Main Target Y
                * 4 {3}  = Main Target Z
                * 5 {4}  = Colour output?
                * 6 {5}  = Command Request
                * 7 {6}  = Command Distance
                * 8 {7}  = Ignore Distance
                * ---
                * 9  -   = GPS Data 8
                * 10 -   = GPS Data 9
                * 11 {0} = Alignment Target X
                * 12 {1} = Alignment Target Y
                * 13 {2} = Alignment Target Z
                * 
                */

            c.Clear().EnsureCapacity(align_target_valid ? 120 : 80); // ~80 chars base, ~40 more if asteroid
            c.AppendFormat(baseFormat, currentDroneBoreIndex, currentMiningDroneBoreLocationX, currentMiningDroneBoreLocationY, currentMiningDroneBoreLocationZ, currentMiningDroneCustomData_5, commandOutput, currentMiningDroneBoreLength, currentMiningDroneIgnoreDistance);
            if (align_target_valid) c.AppendFormat(astFormat, currentMiningDroneAlignLocationX, currentMiningDroneAlignLocationY, currentMiningDroneAlignLocationZ);
        }
        void transmit_to_drone()
        {
            IGC.SendBroadcastMessage(tx_chan, droneTranmissionOutput[di], TransmissionDistance.TransmissionDistanceMax);
        }
        void GetStoredData()
        {
            if (!string.IsNullOrEmpty(Storage)) // Check null or empty
            {
                gridBoreDataList.Clear();
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


                        if (str_datai.Length >= 5) // Full bn:bc:x:y:z
                        {
                            double x = double.TryParse(str_datai[2], out bx) ? bx : 0.0;
                            double y = double.TryParse(str_datai[3], out by) ? by : 0.0;
                            double z = double.TryParse(str_datai[4], out bz) ? bz : 0.0;
                            gridBoreDataList.Add(new GridBoreData(new Vector3D(x, y, z), bcParsed && bc > 0, bnParsed && bn > 0));
                        }
                        else
                        {
                            gridBoreDataList.Add(new GridBoreData(new Vector3D(0, 0, 0), false, true)); // Default position if incomplete
                        }
                    }
                }
            }
        }
        void reset_drone_data()
        {
            drone_name.Clear();
            drone_damage_state.Clear();
            drone_tunnel_complete.Clear();
            drone_control_status.Clear();
            drone_dock_status.Clear();
            drone_undock_status.Clear();
            drone_autopilot_status.Clear();
            drone_gps_grid_list_position.Clear();
            drone_gps_coordinates_ds.Clear();
            drone_drill_depth_value.Clear();
            drone_mine_distance_status.Clear();
            drone_mine_depth_start_status.Clear();
            drone_location_x.Clear();
            drone_location_y.Clear();
            drone_location_z.Clear();
            drone_charge_storage.Clear();
            drone_gas_storage.Clear();
            drone_ore_storage.Clear();
            drone_mining.Clear();
            drone_assigned_coordinates.Clear();
            drone_control_sequence.Clear(); ;
            drone_recall_sequence.Clear();
            droneTranmissionOutput.Clear();
            drone_ready.Clear();
            drone_must_wait.Clear();
            dcs.Clear();
            droneStatusOK.Clear();
            droneTransmissionStatus.Clear();
            drone_recall_list.Clear();
            drone_reset_func.Clear();
            drone_assigns_count.Clear();
        }
        void reset_drone_list()
        {
            drone_name = new List<string>();
            drone_damage_state = new List<string>();
            drone_tunnel_complete = new List<string>();
            drone_control_status = new List<string>();
            drone_dock_status = new List<string>();
            drone_undock_status = new List<string>();
            drone_autopilot_status = new List<string>();
            drone_gps_grid_list_position = new List<int>();
            drone_gps_coordinates_ds = new List<Vector3D>();
            drone_drill_depth_value = new List<string>();
            drone_mine_distance_status = new List<string>();
            drone_mine_depth_start_status = new List<string>();
            drone_location_x = new List<string>();
            drone_location_y = new List<string>();
            drone_location_z = new List<string>();
            drone_charge_storage = new List<string>();
            drone_gas_storage = new List<string>();
            drone_ore_storage = new List<string>();
            drone_mining = new List<bool>();
            drone_assigned_coordinates = new List<bool>();
            drone_control_sequence = new List<int>();
            drone_recall_sequence = new List<int>();
            droneTranmissionOutput = new List<string>();
            drone_ready = new List<bool>();
            drone_must_wait = new List<bool>();
            dcs = new List<double>();
            droneStatusOK = new List<bool>();
            droneTransmissionStatus = new List<bool>();
            drone_recall_list = new List<bool>();
            drone_reset_func = new List<bool>();
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
        void state_shifter()
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


            if (at_all[index] != null)
            {
                at_all[index].CustomData = $"{drone_tag}:{secondary}:";
            }
            Echo($"Drone info:{drone_tag}");
            ant_tg = "[" + drone_tag + " " + comms + "]";
            lt_tag = "[" + drone_tag + " " + comms + "]";
            dp_mn_tag = "[" + drone_tag + " " + MainS + " " + dspy + "]";
            dp_drn_tag = "[" + drone_tag + " " + DroneS + " " + dspy + "]";
            dp_lst_tag = "[" + drone_tag + " " + LstS + " " + dspy + "]";
            dp_vis_tag = "[" + drone_tag + " " + GrphS + " " + dspy + "]";
            intfc_tag = "[" + drone_tag + " " + IntfS + "]";
            secondary_tag = "[" + secondary + "]";
            if (secondary == "" || secondary == " " || secondary == null)
            {
                secondary_tag = "";
            }
            rx_ch = drone_tag + " " + replyC;
            rx_ch_2 = drone_tag + " " + prospC;
            tx_recall_channel = drone_tag + " " + command_recall;
            tx_ping_channel = "[" + drone_tag + "]" + " " + p_cht;
            Me.CustomName = $"GMDC Programmable Block {secondary_tag} {ant_tg}";
        }

        public void setup_system()
        {
            #region setup_system
            IMyGridTerminalSystem gts = GridTerminalSystem as IMyGridTerminalSystem;
            ant_tg = "[" + drone_tag + " " + comms + "]";
            lt_tag = "[" + drone_tag + " " + comms + "]";
            dp_mn_tag = "[" + drone_tag + " " + MainS + " " + dspy + "]";
            dp_drn_tag = "[" + drone_tag + " " + DroneS + " " + dspy + "]";
            dp_lst_tag = "[" + drone_tag + " " + LstS + " " + dspy + "]";
            intfc_tag = "[" + drone_tag + " " + IntfS + "]";
            dp_vis_tag = "[" + drone_tag + " " + GrphS + " " + dspy + "]";
            secondary_tag = "[" + secondary + "]";
            rx_ch = drone_tag + " " + replyC;
            rx_ch_2 = drone_tag + " " + prospC;
            tx_recall_channel = drone_tag + " " + command_recall;
            tx_ping_channel = "[" + drone_tag + "]" + " " + p_cht;
            drone_location = new List<Vector3D>();
            drone_name = new List<string>();
            drone_damage_state = new List<string>();
            drone_tunnel_complete = new List<string>();
            drone_control_status = new List<string>();
            drone_dock_status = new List<string>();
            drone_undock_status = new List<string>();
            drone_autopilot_status = new List<string>();
            drone_gps_coordinates_ds = new List<Vector3D>();
            drone_control_sequence = new List<int>();
            drone_gps_grid_list_position = new List<int>();
            drone_assigned_coordinates = new List<bool>();            
            drone_drill_depth_value = new List<string>();
            drone_mine_distance_status = new List<string>();
            drone_mine_depth_start_status = new List<string>();
            drone_mining = new List<bool>();
            drone_location_x = new List<string>();
            drone_location_y = new List<string>();
            drone_location_z = new List<string>();
            drone_charge_storage = new List<string>();
            drone_gas_storage = new List<string>();
            drone_ore_storage = new List<string>();
            droneTranmissionOutput = new List<string>();
            drone_recall_sequence = new List<int>();
            drone_ready = new List<bool>();
            drone_must_wait = new List<bool>();
            drone_recall_list = new List<bool>();
            drone_assigns_count = new List<int>();
            sprites = new List<MySprite>();
            rm_ctl_all = new List<IMyRemoteControl>();
            rm_ctl_tag = new List<IMyRemoteControl>();
            drone_cargo_full = new List<string>();
            drone_recharge_request = new List<string>();
            drone_auto_pilot_enabled = new List<string>();
            gridBoreDataList = new List<GridBoreData>();
            droneDataList = new List<DroneData>();
            sb = new StringBuilder();
            bores_regen = false;
            cl = new List<string>();
            cl2 = new List<string>();
            tla = new List<int>();
            rst = new List<int>();
            fct = new List<string>();
            dcs = new List<double>();
            droneStatusOK = new List<bool>();
            droneTransmissionStatus = new List<bool>();
            miningCoordinatesNew = new StringBuilder();
            dp_txm = new StringBuilder();
            dp_txl = new StringBuilder();
            droneInformation = new StringBuilder();
            drone_reset_func = new List<bool>();
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
            at_all = new List<IMyRadioAntenna>();
            at_tg = new List<IMyRadioAntenna>();
            gts.GetBlocksOfType<IMyRadioAntenna>(at_all, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < at_all.Count; i++)
            {
                if (at_all[i].CustomName.Contains(ant_tg) || at_all[i].CustomName.Contains(comms))
                {
                    string checker = at_all[i].CustomData;
                    drone_custom_data_check(checker, i);
                    if (drone_tag == "" || drone_tag == null)
                    {
                        Echo($"Invalid name for drone_tag {drone_tag} please add drone tag to GMDC antenna custom data '<yourdronetaghere>:<Yourshiptaghere>:' e.g. 'SWRM_D:Atlas:'");
                        return;
                    }
                    at_all[i].CustomName = $"GMDC Antenna {secondary_tag} {ant_tg}";
                    at_tg.Add(at_all[i]);
                }
            }
            at_all.Clear();
            gts.GetBlocksOfType<IMyRemoteControl>(rm_ctl_all, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < rm_ctl_all.Count; i++)
            {
                if (rm_ctl_all[i].CustomName.Contains(ant_tg) || rm_ctl_all[i].CustomName.Contains(comms))
                {
                    rm_ctl_all[i].CustomName = $"GMDC Remote Control {secondary_tag} {ant_tg}";
                    rm_ctl_tag.Add(rm_ctl_all[i]);
                }
            }
            rm_ctl_all.Clear();

            lts_all = new List<IMyLightingBlock>();
            lts_sys_tg = new List<IMyLightingBlock>();
            gts.GetBlocksOfType<IMyLightingBlock>(lts_all, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < lts_all.Count; i++)
            {
                if (lts_all[i].CustomName.Contains(lt_tag) || lts_all[i].CustomName.Contains(comms))
                {
                    lts_all[i].CustomName = $"GMDC Indicator Light {secondary_tag} {lt_tag}";
                    lts_sys_tg.Add(lts_all[i]);
                }
            }
            lts_all.Clear();
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
            pb_all = new List<IMyProgrammableBlock>();
            pb_tg = new List<IMyProgrammableBlock>();
            gts.GetBlocksOfType<IMyProgrammableBlock>(pb_all, b => b.CubeGrid == Me.CubeGrid);
            for (int i = 0; i < pb_all.Count; i++)
            {
                if (pb_all[i].CustomName.Contains(intfc_tag) || pb_all[i].CustomName.Contains(IntfS))
                {
                    pb_all[i].CustomName = $"GMDI Programmable Block {secondary_tag} {intfc_tag}";
                    pb_tg.Add(pb_all[i]);
                }
            }
            pb_all.Clear();

            drone_messages_list = new List<MyIGCMessage>();
            prospector_messages_list = new List<MyIGCMessage>();

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

        public void presence_check()
        {
            #region presence_check
            if (at_tg.Count <= 0 || at_tg[0] == null)
            {
                Echo($"Antenna with tag: '{ant_tg}' not found.");
                return;
            }
            ant_act = at_tg[0];
            if (rm_ctl_tag.Count <= 0 || rm_ctl_tag[0] == null)
            {
                Echo($"remote control with tag: '{ant_tg}' not found.");
                return;
            }
            remote_control_actual = rm_ctl_tag[0];


            if (lts_sys_tg.Count <= 0 || lts_sys_tg[0] == null)
            {
                Echo($"light with tag: '{lt_tag}' not found.");
                return;
            }
            light_status_indicator_actual = lts_sys_tg[0];
            light_status_indicator_actual.SetValue("Color", Cred);
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
            if (pb_tg.Count <= 0 || pb_tg[0] == null)
            {
                Echo($"Interface PB with tag: '{intfc_tag}' not found.");
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

        public struct DroneData
        {
            /*Format = "{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:{8}:{9}:{10}:{11}:{12}:{13}:{14}:{15}:{16}:{17}:{18}:{19}:{20}:";
            * 0 = D_I_N (drone name)
            * 1 = droneDamageStatus
            * 2 = tunnelSequenceFinished
            * 3 = droneStatusOutput
            * 4 = is_docked
            * 5 = isUndocked
            * 6 = isAutopiloting
            * 7 = rc_actual.IsAutoPilotEnabled
            * 8 = Math.Round(rc_xyz.X, 2) (current drone location x)
            * 9 = Math.Round(rc_xyz.Y, 2) (current drone location y)
            * 10 = Math.Round(rc_xyz.Z, 2) (current drone location z)
            * 11 = drillSetLength (drill commanded distance)
            * 12 = Math.Round(distance_current, 2) (current drill distance)
            * 13 = Math.Round(drillSetLength - ignoreDistance, 2) (current drill start distance)
            * 14 = Math.Round(percent_battery_power, 2) (current drone charge)
            * 15 = Math.Round(pcnt_gas_tank, 2) (current drone gas)
            * 16 = Math.Round(total_percent_cargo_used, 2) (current drone ore)
            * 17 = gpsIndex (current drone gps index)
            * 18 = cargoFullAchieved (current drone cargo full status)
            * 19 = recharge_request (current drone recharge request)
            * 20 = autoDock (current drone auto pilot enabled)
            * 
            */
            public string droneName;
            public string droneDamage;
            public bool droneTunnelFinished;
            public string droneStatusOutput;
            public bool isDocked;
            public bool isUndocked;
            public bool isAutopiloting;
            public bool rcAutopilotEnabled;
            public double droneLocationX;
            public double droneLocationY;
            public double droneLocationZ;
            public double currentBoreLength;
            public double currentBoreDistance;
            public double currentBoreMineDistance;
            public double currentDroneCharge;
            public double currentDroneGas;
            public double currentDroneOre;
            public int currentGPSIndex;
            public bool cargoFull;
            public bool rechargeRequest;
            public bool autoDock;

            public DroneData(
                string droneName, string droneDamage, bool droneTunnelFinished, 
                string droneStatusOutput, bool isDocked, bool isUndocked, 
                bool isAutopiloting, bool rcAutopilotEnabled, double droneLocationX, 
                double droneLocationY, double droneLocationZ, double currentBoreLength, 
                double currentBoreDistance, double currentBoreMineDistance, double currentDroneCharge, 
                double currentDroneGas, double currentDroneOre, int currentGPSIndex,
                bool cargoFull, bool rechargeRequest, bool autoDock
                )
            {
                this.droneName = droneName;
                this.droneDamage = droneDamage;
                this.droneTunnelFinished = droneTunnelFinished;
                this.droneStatusOutput = droneStatusOutput;
                this.isDocked = isDocked;
                this.isUndocked = isUndocked;
                this.isAutopiloting = isAutopiloting;
                this.rcAutopilotEnabled = rcAutopilotEnabled;
                this.droneLocationX = droneLocationX;
                this.droneLocationY = droneLocationY;
                this.droneLocationZ = droneLocationZ;
                this.currentBoreLength = currentBoreLength;
                this.currentBoreDistance = currentBoreDistance;
                this.currentBoreMineDistance = currentBoreMineDistance;
                this.currentDroneCharge = currentDroneCharge;
                this.currentDroneGas = currentDroneGas;
                this.currentDroneOre = currentDroneOre;
                this.currentGPSIndex = currentGPSIndex;
                this.cargoFull = cargoFull;
                this.rechargeRequest = rechargeRequest;
                this.autoDock = autoDock;
            }
        }

        public void recieved_drone_message_to_database()
        {
            #region drone_message_data_processing
            if (recieved_drone_id_name != null && recieved_drone_id_name != "")
            {
                found = false;
                //drone does not exist assume none and add first
                if (drone_name.Count <= 0)
                {
                    drone_name.Add(recieved_drone_id_name);
                    drone_damage_state.Add(rc_dds);
                    drone_tunnel_complete.Add(rc_tnl_end);
                    drone_control_status.Add(rc_dn_sts);
                    drone_dock_status.Add(recieved_drone_dock);
                    drone_undock_status.Add(recieved_drone_undock);
                    drone_autopilot_status.Add(recived_drone_autopilot);

                    drone_cargo_full.Add(rc_dn_cargo_full);
                    drone_recharge_request.Add(rc_dn_rchg_req);
                    drone_auto_pilot_enabled.Add(rc_auto_pilot_enabled);
                    drone_gps_coordinates_ds.Add(remote_control_actual.GetPosition());
                    drone_drill_depth_value.Add(rc_dn_drl_dpth);
                    drone_mine_distance_status.Add(rc_dn_drl_crnt);
                    drone_mine_depth_start_status.Add(rc_dn_drl_strt);
                    drone_location_x.Add(rc_locx);
                    drone_location_y.Add(rc_locy);
                    drone_location_z.Add(rc_locz);
                    drone_charge_storage.Add(rc_dn_chg);
                    drone_gas_storage.Add(rc_dn_gas);
                    drone_ore_storage.Add(rc_dn_str);
                    drone_gps_grid_list_position.Add(-1);
                    drone_mining.Add(false);
                    drone_assigned_coordinates.Add(false);
                    drone_control_sequence.Add(0);
                    drone_recall_sequence.Add(0);
                    droneTranmissionOutput.Add("");
                    drone_ready.Add(false);
                    drone_must_wait.Add(true);
                    dcs.Add(0.0);
                    droneStatusOK.Add(true);
                    droneTransmissionStatus.Add(true);
                    drone_recall_list.Add(false);
                    drone_reset_func.Add(false);
                    drone_assigns_count.Add(0);

                }
                //drones do exist so check current drone list to see if it exists
                if (drone_name.Count > 0)
                {

                    for (int i = 0; i < drone_name.Count; i++)
                    {
                        found = false;
                        string nametag = drone_name[i];
                        //drone name found so update data
                        if (nametag == recieved_drone_id_name)
                        {
                            found = true;
                            drone_name[i] = recieved_drone_id_name;
                            drone_damage_state[i] = rc_dds;
                            drone_tunnel_complete[i] = rc_tnl_end;
                            drone_control_status[i] = rc_dn_sts;
                            drone_dock_status[i] = recieved_drone_dock;
                            drone_undock_status[i] = recieved_drone_undock;
                            drone_autopilot_status[i] = recived_drone_autopilot;
                            drone_drill_depth_value[i] = (rc_dn_drl_dpth);
                            drone_mine_distance_status[i] = rc_dn_drl_crnt;
                            drone_mine_depth_start_status[i] = rc_dn_drl_strt;
                            drone_gps_grid_list_position[i] = recieved_drone_list_position;
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
                            if (dcs[i] <= bclm)
                            {
                                droneStatusOK[i] = false;
                            }
                            if (dcs[i] > bclm)
                            {
                                droneStatusOK[i] = true;
                            }
                            Confirmed_Drone_Message = true;
                            recieved_drone_name_index = i;
                            break;
                        }
                        //drone not found at end of list so add drone to list
                        if (i == (drone_name.Count - 1) && !nametag.Equals(recieved_drone_id_name) && !found)
                        {
                            drone_name.Add(recieved_drone_id_name);
                            drone_damage_state.Add(rc_dds);
                            drone_tunnel_complete.Add(rc_tnl_end);
                            drone_control_status.Add(rc_dn_sts);
                            drone_dock_status.Add(recieved_drone_dock);
                            drone_undock_status.Add(recieved_drone_undock);
                            drone_autopilot_status.Add(recived_drone_autopilot);
                            drone_gps_coordinates_ds.Add(remote_control_actual.GetPosition());
                            drone_drill_depth_value.Add(rc_dn_drl_dpth);
                            drone_mine_distance_status.Add(rc_dn_drl_crnt);
                            drone_mine_depth_start_status.Add(rc_dn_drl_strt);
                            drone_location_x.Add(rc_locx);
                            drone_location_y.Add(rc_locy);
                            drone_location_z.Add(rc_locz);
                            drone_charge_storage.Add(rc_dn_chg);
                            drone_gas_storage.Add(rc_dn_gas);
                            drone_ore_storage.Add(rc_dn_str);
                            drone_mining.Add(false);
                            drone_assigned_coordinates.Add(false);
                            drone_gps_grid_list_position.Add(-1);
                            drone_control_sequence.Add(0);
                            drone_recall_sequence.Add(0);
                            droneTranmissionOutput.Add("");
                            drone_ready.Add(false);
                            drone_must_wait.Add(true);
                            dcs.Add(0.0);
                            droneStatusOK.Add(true);
                            droneTransmissionStatus.Add(true);
                            drone_recall_list.Add(false);
                            drone_reset_func.Add(false);
                            drone_assigns_count.Add(0);
                            drone_cargo_full.Add(rc_dn_cargo_full);
                            drone_recharge_request.Add(rc_dn_rchg_req);
                            drone_auto_pilot_enabled.Add(rc_auto_pilot_enabled);
                            Confirmed_Drone_Message = true;
                            recieved_drone_name_index = i + 1;
                        }
                    }
                }
            }
            #endregion
        }

        int CntTrueVlsGridBoreDataFinished(List<GridBoreData> boreList)
        {
            int count = 0;
            for (int i = 0; i < boreList.Count; i++)
            {
                if (boreList[i].boreFinished) count++;
            }
            return count;
        }
        void update_display()  // Extracted from drone_processing
        {
            dp_txm.Clear().EnsureCapacity(512); // ~400-600 chars typical
            dp_txm.AppendLine($"GMDC {ver} Running {icon}")
                  .AppendLine($"------------------------------")
                  .AppendLine($" ")
                   .AppendLine($"Total drones detected: {drone_name.Count}")
                  .AppendLine(launched_drone_status
                      ? $"Drones active: {total_drones_mining} Idle: {t_drn_idle} Fault: {t_drn_dmg} (Max: {max_active_drone_count} ({flight_factor})) Hard limit: {hard_active_drone_limit}"
                      : $"Drones active: {total_drones_mining} Idle: {t_drn_idle} Fault: {t_drn_dmg}")
                  .AppendLine($"Docking: {t_drn_dckg} Docked: {t_drn_dck} - Recharge: {t_drn_rechg} Unload: {t_drn_unload}")
                  .AppendLine($"Undocking: {t_drn_udckg} Undocked: {t_drn_udck} - Mining: {t_drn_mine} Exit: {t_drn_exit}")
                  .AppendLine()
                  .AppendLine($"Surface distance: {safe_dstvl}m")
                  .AppendLine($"Drill depth: {drillLength}m ({drillLength + safe_dstvl}m)")
                  .AppendLine($"Req. ignore depth: {ignoreDepth}m (Drone length: {drone_length}m)")
                  .AppendLine($"Ignore depth: {safe_dstvl + drone_length + ignoreDepth}m (Drill Start: {(drillLength + safe_dstvl) - (ignoreDepth + safe_dstvl + drone_length)}m)")
                  .AppendLine()
                  .AppendLine($"Command: {commandAsk} Reset: {general_reset}")
                  .AppendLine($"Status: {stts}")
                  .AppendLine()
                  .AppendLine("Target:")
                  .AppendLine(mainGPSCoordinates.ToString());
            if (align_target_valid) dp_txm.AppendLine("Secondary/Asteroid:").AppendLine(alignGPSCoordinates.ToString());

            if (display_tag_main.Count > 0 && sM != null) sM.WriteText(dp_txm);
        }

        //program end

    }
}

