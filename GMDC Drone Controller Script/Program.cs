﻿using Sandbox.Game.EntityComponents;
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
        string ver = "V0.386B";
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
        string data_in_drone;
        string data_in_prospector;
        double bclm = 1.0;
        string recievedDroneName;
        string recievedDroneDamageState;
        string recievedDroneTunnelFinished;
        string receivedDroneControlStatus;
        string receievdDroneDockStatus;
        string recievedDroneUndockStatus;
        string recievedDroneAutopilot;
        string receievdDroneDrillDepth;
        string receivedDroneMinedDistance;
        string recivedDroneMineStart;
        string recievedDroneListPosition;
        string recivedDroneCargoFull;
        string recievedDroneRechargeRequest;
        string recievedDroneAutopilotEnabled;
        int _recievedDroneListPosition;
        double rc_d_cn = 0.0;
        string recievedDroneLocationX;
        string recievedDroneLocationY;
        string recievedDroneLocationZ;
        string recivedDroneCharge;
        string recievedDroneGas;
        string recivedDroneOre;
        string recievedDroneAutoDockEnabled;
        string recievedDroneDockingReady;
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
        bool flto = false;
        int fltc = 0;
        private IEnumerator<bool> gridCoroutine;
        private IEnumerator<bool> listCoroutine;
        private IEnumerator<bool> visCoroutine;
        IMyRadioAntenna ant_act;
        IMyLightingBlock light_status_indicator_actual;
        IMyRemoteControl remote_control_actual;
        IMyProgrammableBlock pb_i_act;
        Vector3D m_gps_crds;
        Vector3D c_gps_crds;
        Vector3D next_gps_crds;
        Vector3D target_gps_coords;
        Vector3D align_gps_coords;
        Vector3D planeNrml;
        StringBuilder miningCoordinatesNew;
        StringBuilder c;



        List<droneData> droneDataList;
        List<gridData> gridDataList;
        List<double> dcs;        
        List<string> cl;
        List<string> cl2;
        List<int> tla;
        List<int> rst;
        List<string> fct;
        List<bool> dst;
        //int cbval = 0;
        //bool clbt = false;
        int bores_completed;
        int gps_grid_position_value = -1;
        string drone_namer = "";
        StringBuilder droneInformation;
        StringBuilder dp_txm;
        StringBuilder dp_txl;
        StringBuilder jxt;
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
        //decimal dps_r_d = 0.0m;
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
        string g1;
        string g2;
        int di = 0;
        int total_drones_undocking = 0;
        int undock_timer = 0;
        bool drones_undocking = false;
        bool can_loading = false;
        string px = "";
        string py = "";
        string pz = "";
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
        int recievedDroneNameIndex = -1;
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
            if (setup_complete)
            {
                sb = new StringBuilder();
                if (gridDataList.Count > 0)
                {
                    for (int i = 0; i < gridDataList.Count; i++)
                    {
                        
                        if (gridDataList[i].finished)
                        {
                            g1 = "1";
                        }
                        else
                        {
                            g1 = "0";
                        }
                        if (gridDataList[i].occupied)
                        {
                            g2 = "1";
                        }
                        else
                        {
                            g2 = "0";
                        }
                        px = gridDataList[i].gps_coords.X.ToString();
                        py = gridDataList[i].gps_coords.Y.ToString();
                        pz = gridDataList[i].gps_coords.Z.ToString();
                        sb.Append(g1 + ":" + g2 + ":" + px + ":" + py + ":" + pz + ":" + ";");
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
            //Echo("Post-UpdateRuntimeMetrics");
            InitializeSystem();
            //Echo("Post-InitializeSystem");
            ProcessInputs(argument);
            //Echo("Post-ProcessInputs");
            ManageCommunications();
            //Echo("Post-ManageCommunications");
            UpdateMiningGrid();
            //Echo("Post-UpdateMiningGrid");
            HandleDroneOperations();
            //Echo("Post-HandleDroneOperations");
            RenderDisplays();
            //Echo("Post-RenderDisplays");
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
            if (droneDataList.Count > 0 && created_grid && time_delay)
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
            Echo($"Drones #: {droneDataList.Count}");
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
                    visCoroutine = BuildSprites(m_gps_crds, planeNrml, gridSize, nPtsX, nPtsY, core_out);
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
                        BuildSprites(m_gps_crds, planeNrml, gridSize, nPtsX, nPtsY, core_out).Dispose();
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
                if (display_tag_vis.Count > 0 && display_tag_vis[0] != null && gridDataList.Count > 0 && frame_generator_finished)
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

            if (total_drones_mining > 0 && bores_completed < total_mining_runs && canTransmit && can_run || flto)
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
                if (flto)
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
            if (droneDataList.Count > 0 && canReset)
            {
                reset_status_count = CntIntVlsDroneDataInt(droneDataList,d => d.droneGPSListPosition, -1);
                docked_status_count = CntStsVlstDroneData(droneDataList,d => d.droneControlStatus, "Docked");
            }
            if (droneDataList.Count > 0)
            {
                if (reset_status_count == droneDataList.Count && docked_status_count == droneDataList.Count && canReset)
                {
                    readyFlag = true;
                }
            }
        }

        private void ProcessDroneState()
        {
            int startInstructions = Runtime.CurrentInstructionCount;

            #region drone_state_machine_management            
            
            if (recievedDroneNameIndex != -1 && Confirmed_Drone_Message && droneDataList.Count > 0 && recievedDroneNameIndex < droneDataList.Count)
            {                
                int i = recievedDroneNameIndex;                
                droneData droneInfo = droneDataList[i];
                

                if (can_init || canReset || (droneDataList[i].droneResetFunc && droneDataList.Count >0) || can_loading)
                {
                    general_reset = true;
                }
                else general_reset = false;
                di = i;                
                fltc = CntTrueVls(dst);
                if (fltc < droneDataList.Count)
                {
                    flto = true;
                }
                else flto = false;                
                //recall sequence reset - global
                if (!droneDataList[i].droneRecallList || canReset || can_init || can_loading)
                {
                    droneInfo.droneRecallSequence = 0;
                    droneDataList[i] = droneInfo;                    
                }
                dp_txm.Clear();                
                if (droneDataList[i].droneGPSListPosition > -1 && !droneDataList[i].droneAssignedCoordinates)
                {                    
                    droneInfo.droneGPSListPosition = -1;
                    droneDataList[i] = droneInfo;                    
                }
                //if undocked request local recall sequence flag to ON
                if (droneDataList[i].droneGPSListPosition == -1 && !droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneUndockStatus && droneDataList[i].droneDockStatus && !droneDataList[i].droneRecallList && !mustUndock_Command || droneDataList[i].droneGPSListPosition == -1 && !droneDataList[i].droneAssignedCoordinates && !droneDataList[i].droneUndockStatus && droneDataList[i].droneDockStatus && !droneDataList[i].droneRecallList && !mustUndock_Command)
                {            
                    
                    droneInfo.droneRecallList = true;
                    droneDataList[i] = droneInfo;                    
                }                
                if (droneDataList[i].droneRecallList)
                {
                    
                    tx_drone_recall_channel = droneDataList[i].droneName + " " + command_recall;
                    IGC.SendBroadcastMessage(tx_drone_recall_channel, command_recall, TransmissionDistance.TransmissionDistanceMax);
                }
                if (!droneDataList[i].droneRecallList)
                {
                    tx_drone_recall_channel = droneDataList[i].droneName + " " + command_recall;
                    IGC.SendBroadcastMessage(tx_drone_recall_channel, command_operate, TransmissionDistance.TransmissionDistanceMax);
                }
                
                if (droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneGPSListPosition == -1 && droneDataList[i].droneMining && droneDataList[i].droneControlSequence == 0)
                {
                    droneInfo.droneMining = false;
                    droneDataList[i] = droneInfo;
                }
                



                if (total_drones_mining >= bores_remaining && !droneDataList[i].droneMining && bores_completed <= total_mining_runs || bores_remaining == 0 && !droneDataList[i].droneMining)
                {
                    if (!launched_drone_status || drones_undocking)
                    {
                        droneInfo.droneMustWait = true;
                        droneDataList[i] = droneInfo;                        
                        
                    }
                    if (launched_drone_status && total_drones_mining > max_active_drone_count || drones_undocking)
                    {
                        droneInfo.droneMustWait = true;
                        droneDataList[i] = droneInfo;                        
                    }
                    if (launched_drone_status && total_drones_mining <= max_active_drone_count)
                    {
                        droneInfo.droneMustWait = false;
                        droneDataList[i] = droneInfo;                        
                    }
                }
                
                else if (total_drones_mining < bores_remaining && bores_completed < total_mining_runs || droneDataList[i].droneMining && total_drones_mining <= bores_remaining)
                {
                    if (!launched_drone_status)
                    {
                        droneInfo.droneMustWait = false;
                        droneDataList[i] = droneInfo;
                    }
                    if (launched_drone_status && total_drones_mining < max_active_drone_count)
                    {
                        droneInfo.droneMustWait = false;
                        droneDataList[i] = droneInfo;
                    }

                    if (launched_drone_status && total_drones_mining > max_active_drone_count || drones_undocking)
                    {
                        droneInfo.droneMustWait = true;
                        droneDataList[i] = droneInfo;
                        
                    }
                }                
                if (droneDataList[i].droneGPSListPosition == -1 && total_drones_mining >= bores_remaining || drones_undocking)
                {
                    if (!launched_drone_status)
                    {
                        droneInfo.droneMustWait = true;
                        droneDataList[i] = droneInfo;
                    }
                    if (launched_drone_status && total_drones_mining >= max_active_drone_count || drones_undocking)
                    {
                        droneInfo.droneMustWait = true;
                        droneDataList[i] = droneInfo;
                    }
                    if (launched_drone_status && total_drones_mining < max_active_drone_count || drones_undocking)
                    {
                        droneInfo.droneMustWait = true;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneGPSListPosition > -1 && droneDataList[i].droneGPSListPosition < gridDataList.Count)
                {
                    if (gridDataList[droneDataList[i].droneGPSListPosition].occupied && !droneDataList[i].droneMining)
                    {
                        if (!launched_drone_status || drones_undocking)
                        {
                            droneInfo.droneMustWait = true;
                            droneDataList[i] = droneInfo;
                        }
                        if (launched_drone_status && total_drones_mining >= max_active_drone_count || drones_undocking)
                        {
                            droneInfo.droneMustWait = true;
                            droneDataList[i] = droneInfo;
                        }
                        if (launched_drone_status && total_drones_mining < max_active_drone_count || drones_undocking)
                        {
                            droneInfo.droneMustWait = true;
                            droneDataList[i] = droneInfo;
                        }
                    }
                    else if (bores_completed < total_mining_runs && !gridDataList[droneDataList[i].droneGPSListPosition].occupied && !gridDataList[droneDataList[i].droneGPSListPosition].finished && !droneDataList[i].droneMining)
                    {
                        if (!launched_drone_status)
                        {
                            droneInfo.droneMustWait = false;
                            droneDataList[i] = droneInfo;                            
                        }
                        if (launched_drone_status && total_drones_mining < max_active_drone_count)
                        {
                            droneInfo.droneMustWait = false;
                            droneDataList[i] = droneInfo;                            
                        }
                        if (launched_drone_status && total_drones_mining >= max_active_drone_count || drones_undocking)
                        {
                            droneInfo.droneMustWait = true;
                            droneDataList[i] = droneInfo;                            
                        }
                    }
                    if (!gridDataList[droneDataList[i].droneGPSListPosition].finished)
                    {
                        int queued_count = CntIntVlsDroneDataInt(droneDataList,d=>d.droneGPSListPosition, droneDataList[i].droneGPSListPosition);
                        if (gridDataList[droneDataList[i].droneGPSListPosition].occupied && queued_count == 0)
                        {
                            gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                            gridnew.occupied = false;
                            gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;
                        }

                    }
                }
                
                if (droneDataList[i].droneControlSequence == 12)
                {
                    gps_grid_position_value = -1;
                    droneInfo.droneMining = false;
                    droneDataList[i] = droneInfo;
                }
                if (current_gps_idx > total_mining_runs || !mining_grid_valid && bores_completed >= total_mining_runs || bores_remaining == 0)
                {
                    can_run = false;
                    droneInfo.droneControlSequence = 12;
                    droneDataList[i] = droneInfo;
                    current_gps_idx = 0;
                }

                updateDisplay(i);



                gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                if (droneDataList[i].droneControlStatus == "Docked Idle")
                {
                    droneInfo.droneReady = true;
                    droneDataList[i] = droneInfo;
                    
                }
                if (droneDataList[i].droneControlStatus.Contains("Recharging") || droneDataList[i].droneControlStatus.Contains("Unloading"))
                {
                    droneInfo.droneReady = false;
                    droneDataList[i] = droneInfo;
                }
                if (droneDataList[i].droneReady && !droneDataList[i].droneTunnelFinished && droneDataList[i].droneDockStatus && can_run && !droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneControlSequence == 0 && !droneDataList[i].droneMustWait && !droneDataList[i].droneMining && !run_arg)
                {
                    if (bores_completed < total_mining_runs && mining_grid_valid != false && !droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMustWait == false)
                    {
                           
                        if (gridDataList.Count > 0)
                        {
                            if (skip_bores_number > gridDataList.Count)
                            {
                                skip_bores_number = 0;
                            }
                            if (skip_bores_number > 0)
                            {
                                for (int j = 0; j < skip_bores_number; j++)
                                {
                                    if (j > gridDataList.Count - 1 || j > skip_bores_number - 1)
                                    {
                                        break;
                                    }
                                    gridData gridnew = gridDataList[j];
                                    gridnew.finished = true;
                                    gridDataList[j] = gridnew;
                                }
                            }
                            for (int k = 0; k < gridDataList.Count; k++)
                            {

                                if (k > gridDataList.Count - 1)
                                {
                                    k = gridDataList.Count - 1;
                                }
                                if (!gridDataList[k].finished && !gridDataList[k].occupied)
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
                            droneInfo.droneGPSCoordinatesDS = gridDataList[gps_grid_position_value].gps_coords;                            
                            droneInfo.droneGPSListPosition = gps_grid_position_value;                            
                            droneDataList[i] = droneInfo;
                        }
                        else
                        {
                            gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                            droneInfo.droneGPSCoordinatesDS = gridDataList[gps_grid_position_value].gps_coords;                            
                            droneDataList[i] = droneInfo;
                        }
                        if (!mining_grid_valid)
                        {
                            total_mining_runs = 1;
                            droneInfo.droneGPSCoordinatesDS = m_gps_crds;                            
                            droneDataList[i] = droneInfo;
                            gps_grid_position_value = 0;
                            current_gps_idx = 0;
                        }
                        //suspect code here
                        Echo($"Drone coords: {i}");
                        droneInfo.droneAssignedCoordinates = true;
                        droneDataList[i] = droneInfo;                        
                        Echo($"Drone coords assigned: {i} {droneDataList[i].droneAssignedCoordinates}");
                    }
                    else if (!mining_grid_valid)
                    {
                        total_mining_runs = 1;
                        droneInfo.droneGPSCoordinatesDS = m_gps_crds;                        
                        droneInfo.droneAssignedCoordinates = true;                        
                        droneDataList[i] = droneInfo;
                        gps_grid_position_value = 0;
                        current_gps_idx = 0;
                    }
                    if (droneDataList[i].droneGPSListPosition > -1)
                    {
                        if (gridDataList[droneDataList[i].droneGPSListPosition].occupied && !droneDataList[i].droneMining)
                        {
                            droneInfo.droneMustWait = true;
                            droneDataList[i] = droneInfo;                            
                        }
                        else if (total_drones_mining < bores_remaining && bores_completed < total_mining_runs || !gridDataList[droneDataList[i].droneGPSListPosition].occupied && !gridDataList[droneDataList[i].droneGPSListPosition].finished && !droneDataList[i].droneMining)
                        {
                            droneInfo.droneMustWait = false;
                            droneDataList[i] = droneInfo;                            
                        }
                        if (bores_completed != total_mining_runs && !droneDataList[i].droneMustWait)
                        {                            
                            droneInfo.droneControlSequence = 1;
                            droneInfo.droneMining = true;
                            droneDataList[i] = droneInfo;
                            gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                            gridnew.occupied = true;
                            gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;
                        }
                        else
                        {
                            droneInfo.droneControlSequence = 0;                                                  
                            droneInfo.droneMining = false;
                            droneDataList[i] = droneInfo;
                        }
                        if (gridDataList[droneDataList[i].droneGPSListPosition].finished)
                        {
                            //suspect coordinates here 2
                            Echo($"Drone position finished {i}");
                            droneInfo.droneControlSequence = 0;                            
                            droneInfo.droneMining = false;
                            droneInfo.droneAssignedCoordinates = false;                            
                            droneInfo.droneGPSListPosition = -1;
                            droneDataList[i] = droneInfo;
                        }
                    }
                }
                tx_chan = droneDataList[i].droneName;
                cd1 = gps_grid_position_value.ToString();
                cm = "0";
                xp = Math.Round(droneDataList[i].droneGPSCoordinatesDS.X, 2).ToString();
                yp = Math.Round(droneDataList[i].droneGPSCoordinatesDS.Y, 2).ToString();
                zp = Math.Round(droneDataList[i].droneGPSCoordinatesDS.Z, 2).ToString();
                cd5 = cst_dt5;
                cd6 = (drillLength + safe_dstvl).ToString();
                igd = (ignoreDepth + safe_dstvl + drone_length).ToString();
                if (align_target_valid)
                {
                    xp2 = Math.Round(((droneDataList[i].droneGPSCoordinatesDS.X - m_gps_crds.X) + align_gps_coords.X), 2).ToString();
                    yp2 = Math.Round(((droneDataList[i].droneGPSCoordinatesDS.Y - m_gps_crds.Y) + align_gps_coords.Y), 2).ToString();
                    zp2 = Math.Round(((droneDataList[i].droneGPSCoordinatesDS.Z - m_gps_crds.Z) + align_gps_coords.Z), 2).ToString();
                }
                else
                {
                    xp2 = "";
                    yp2 = "";
                    zp2 = "";
                }
                if (droneDataList[i].droneControlSequence == 1 && droneDataList[i].droneAssignedCoordinates && !droneDataList[i].droneMustWait && !run_arg || droneDataList[i].droneControlSequence == 2 && droneDataList[i].droneControlStatus == "Docked Idle" && droneDataList[i].droneDockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 2;                    
                    droneInfo.droneMining = true;
                    gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                    gridnew.occupied = true;
                    gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "7";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;                        
                        droneDataList[i] = droneInfo;

                    }
                }
                if (droneDataList[i].droneControlSequence == 2 && droneDataList[i].droneControlStatus == "Undocked" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg || droneDataList[i].droneControlSequence == 2 && droneDataList[i].droneControlStatus == "Docking" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 3;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }

                if (droneDataList[i].droneControlSequence == 2 && droneDataList[i].droneControlStatus == "Undocking" && droneDataList[i].droneDockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg && dcs[i] <= bclu )
                {
                    droneInfo.droneControlSequence = 13;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 8 && droneDataList[i].droneControlStatus.Contains("RTB Ready") && droneDataList[i].droneDockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 13;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 13 && droneDataList[i].droneControlStatus == "Idle" && droneDataList[i].droneDockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg || droneDataList[i].droneControlSequence == 5 && droneDataList[i].droneControlStatus == "Docking" && droneDataList[i].droneDockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg && dcs[i] <= bclu)
                {
                    droneInfo.droneControlSequence = 8;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "6";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 3 && droneDataList[i].droneControlStatus == "Idle" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 4;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "4";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 4 && droneDataList[i].droneControlStatus == "Nav End" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {                    
                    droneInfo.droneControlSequence = 5;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 4 && droneDataList[i].droneControlStatus == "Docked Idle" && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 1;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }

                if (droneDataList[i].droneControlSequence == 5 && droneDataList[i].droneControlStatus == "Idle" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 6;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "2";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }

                if (droneDataList[i].droneControlSequence == 6 && droneDataList[i].droneControlStatus == "Nav End" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 7;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 7 && droneDataList[i].droneControlStatus == "Idle" && droneDataList[i].droneUndockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 8;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "5";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence >= 8 && droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneMining || droneDataList[i].droneControlSequence == 4 && droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneMining)
                {
                    gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                    gridnew.occupied = false;
                    gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;
                }
                if (droneDataList[i].droneControlSequence >= 8 && droneDataList[i].droneControlStatus.Contains("Dock") && droneDataList[i].droneMining && droneDataList[i].droneTunnelFinished)
                {
                    gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                    gridnew.finished = true;
                    gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;
                    
                }
                if (droneDataList[i].droneControlSequence == 8 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg)
                {
                    droneInfo.droneControlSequence = 1;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 8 && !droneDataList[i].droneReady && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg || droneDataList[i].droneControlSequence == 8 && !droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg || droneDataList[i].droneControlSequence >= 1 && droneDataList[i].droneControlSequence <= 4 && !droneDataList[i].droneReady && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg)
                {
                    droneInfo.droneControlSequence = 0;                    
                    droneInfo.droneAssignedCoordinates = false;                    
                    droneInfo.droneMining = false;                    
                    gps_grid_position_value = -1;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 8 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg)
                {
                    droneInfo.droneControlSequence = 9; 
                    gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                    gridnew.finished = true;
                    gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;

                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 9 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && can_run && droneDataList[i].droneAssignedCoordinates && !run_arg || droneDataList[i].droneControlSequence == 9 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneAssignedCoordinates && !run_arg)
                {
                    droneInfo.droneControlSequence = 10;                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 10 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && general_reset && droneDataList[i].droneAssignedCoordinates && !run_arg || droneDataList[i].droneControlSequence == 10 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg || droneDataList[i].droneControlSequence == 0 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && !run_arg)
                {
                    droneInfo.droneControlSequence = 11;                    
                    droneInfo.droneTunnelFinished = false;                    
                    gridData gridnew = gridDataList[droneDataList[i].droneGPSListPosition];
                    gridnew.finished = true;
                    gridDataList[droneDataList[i].droneGPSListPosition] = gridnew;
                    t_mne_sq_cmp++;
                    gps_grid_position_value = -1;
                    droneInfo.droneResetFunc = false;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "8";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 11 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && t_mne_sq_cmp <= total_mining_runs && mining_grid_valid && !run_arg)
                {
                    droneInfo.droneControlSequence = 0;                    
                    droneInfo.droneAssignedCoordinates = false;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlSequence == 11 && droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && current_gps_idx < total_mining_runs && droneDataList[i].droneAssignedCoordinates && t_mne_sq_cmp > total_mining_runs && !run_arg || droneDataList[i].droneControlSequence == 11 && droneDataList[i].droneReady && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && droneDataList[i].droneAssignedCoordinates && mining_grid_valid == false && t_mne_sq_cmp >= total_mining_runs && !run_arg)
                {
                    droneInfo.droneControlSequence = 12;                    
                    droneInfo.droneAssignedCoordinates = false;                    
                    gps_grid_position_value = -1;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }

                    dp_txm.Append('\n');
                    dp_txm.Append("Mining seq. complete");
                }
                if (droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && general_reset || droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneDockStatus && droneDataList[i].droneTunnelFinished && general_reset && !run_arg)
                {
                    droneInfo.droneControlSequence = 0;                    
                    t_mne_sq_cmp = 0;
                    droneInfo.droneTunnelFinished = false;
                    droneInfo.droneAssignedCoordinates = false;
                    droneInfo.droneMining = false;                    
                    current_gps_idx = 0;
                    gps_grid_position_value = -1;
                    droneInfo.droneResetFunc = false;
                    
                    cd1 = gps_grid_position_value.ToString();
                    cm = "8";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                if (droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && general_reset && droneDataList[i].droneControlSequence == 0 && !run_arg || droneDataList[i].droneControlStatus.Contains("Docked") && droneDataList[i].droneDockStatus && !droneDataList[i].droneTunnelFinished && general_reset && !run_arg || droneDataList[i].droneControlSequence == 6 && droneDataList[i].droneControlStatus == "Docked Idle" && droneDataList[i].droneDockStatus && droneDataList[i].droneAssignedCoordinates && droneDataList[i].droneMining && !run_arg)
                {
                    droneInfo.droneControlSequence = 0;                    
                    t_mne_sq_cmp = 0;
                    droneInfo.droneTunnelFinished = false;                    
                    droneInfo.droneAssignedCoordinates = false;
                    droneInfo.droneMining = false;                    
                    current_gps_idx = 0;
                    gps_grid_position_value = -1;
                    droneInfo.droneResetFunc = false;
                    cd1 = gps_grid_position_value.ToString();
                    cm = "0";
                    drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                    droneInfo.droneTransmissionOutput = c.ToString();
                    droneDataList[i] = droneInfo;
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }
                
                if (mustRecall_Command && !droneDataList[i].droneRecallList && !mustUndock_Command)
                {
                    droneInfo.droneRecallList = true;
                    droneDataList[i] = droneInfo;                    
                }
                if (droneDataList[i].droneRecallList)
                {
                    if (droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Idle" || droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Undocked" || droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Nav" || droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Undocking" || droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Docking" || droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Initiating mining")
                    {
                        droneInfo.droneRecallSequence = 1;
                        droneDataList[i] = droneInfo;
                    }

                    if (droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus == "Nav End")
                    {
                        droneInfo.droneRecallSequence = 3;
                        droneDataList[i] = droneInfo;
                    }
                    if (droneDataList[i].droneRecallSequence == 1)
                    {
                        droneInfo.droneRecallSequence = 2;
                        droneInfo.droneControlSequence = 0;                        
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "0";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;
                    }
                    if (droneDataList[i].droneRecallSequence == 2 && droneDataList[i].droneControlStatus == "Idle")
                    {
                        droneInfo.droneRecallSequence = 3;
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "1";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;
                    }
                    if (droneDataList[i].droneRecallSequence == 3 && droneDataList[i].droneControlStatus == "Nav End")
                    {
                        droneInfo.droneRecallSequence = 4;
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "0";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;
                    }
                    if (droneDataList[i].droneRecallSequence == 3 && droneDataList[i].droneControlStatus == "Nav" && droneDataList[i].droneGPSListPosition == -1 || droneDataList[i].droneRecallSequence == 3 && droneDataList[i].droneControlStatus == "Idle" && droneDataList[i].droneGPSListPosition >= -1)
                    {
                        droneInfo.droneRecallSequence = 4;
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "0";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;
                    }

                    if (droneDataList[i].droneRecallSequence == 4 && droneDataList[i].droneControlStatus == "Idle")
                    {
                        droneInfo.droneRecallSequence = 5;
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "6";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;
                    }
                    if (droneDataList[i].droneRecallSequence == 4 && droneDataList[i].droneControlStatus == "Idle")
                    {
                        droneInfo.droneRecallSequence = 5;
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "6";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;

                    }
                    if (droneDataList[i].droneRecallSequence == 5 && droneDataList[i].droneControlStatus.Contains("Docked") || droneDataList[i].droneRecallSequence == 0 && droneDataList[i].droneControlStatus.Contains("Docked"))
                    {
                        droneInfo.droneRecallSequence = 0;
                        droneInfo.droneAssignedCoordinates = false;
                        droneInfo.droneRecallList = false;
                        droneInfo.droneMining = false;                        
                        gps_grid_position_value = -1;
                        droneInfo.droneResetFunc = true;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "0";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        droneDataList[i] = droneInfo;
                    }
                    if (canTransmit && droneDataList[i].droneTransmissionStatus)
                    {
                        transmit_to_drone(i);
                        droneInfo.droneTransmissionStatus = false;
                        droneDataList[i] = droneInfo;
                    }
                }

                if (mustUndock_Command)
                {
                    if (droneDataList[i].droneControlStatus == "Docked Idle")
                    {
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "7";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        if (canTransmit && droneDataList[i].droneTransmissionStatus)
                        {
                            transmit_to_drone(i);
                            droneInfo.droneTransmissionStatus = false;
                            droneDataList[i] = droneInfo;
                        }
                    }
                }

                if (mustFreeze_Command)
                {
                    if (droneDataList[i].droneControlStatus == "Undocked" || droneDataList[i].droneControlStatus == "Idle")
                    {
                        gps_grid_position_value = droneDataList[i].droneGPSListPosition;
                        cd1 = gps_grid_position_value.ToString();
                        cm = "0";
                        drone_command_builder(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                        droneInfo.droneTransmissionOutput = c.ToString();
                        
                        if (canTransmit && droneDataList[i].droneTransmissionStatus)
                        {
                            transmit_to_drone(i);
                            droneInfo.droneTransmissionStatus = false;
                            droneDataList[i] = droneInfo;
                        }
                    }
                }
                Confirmed_Drone_Message = false;
                recievedDroneNameIndex = -1;


                if (drone_messages_list.Count > 0)
                {
                    drone_messages_list.RemoveAt(0);
                }


            }
            #endregion
            Echo($"ProcessDroneState: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void ProcessRecallCommand()
        {

            int startInstructions = Runtime.CurrentInstructionCount;
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
            int startInstructions = Runtime.CurrentInstructionCount;
            time_delay = false;
            time_count = 0;
            Echo($"timeCountReset: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        private void UpdateActiveDroneLimits()
        {
            #region active_drones_processing
            //drone limit processing
            if (droneDataList.Count > 0)
            {
                max_active_drone_count = droneDataList.Count - flight_factor;
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

        public struct gridData
        {
            public Vector3D gps_coords;
            public bool occupied;
            public bool finished;

            public gridData(Vector3D gps_coords, bool occupied, bool finished)
            {
                this.gps_coords = gps_coords;
                this.occupied = occupied;
                this.finished = finished;
            }
        }
        private void ProcessJobGrid()
        {
            #region job_grid_processing
            //if mining grid data empty resolve issues to avoid exception
            if (nPtsY == 0 && !created_grid || nPtsX == 0 && !created_grid || gridSize == 0 && !created_grid)
            {
                
                gridDataList = new List<gridData>();

                created_grid = true;                
                c_gps_crds = m_gps_crds;
                gridData gridDatanew = new gridData(c_gps_crds, false, false);
                gridDataList.Add(gridDatanew);

                total_mining_runs = gridDataList.Count;
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
                    gridDataList = new List<gridData>();

                    bores_regen = true;
                }

                if (readyFlag)
                {
                    readyFlag = false;
                }                
                Vector3D gravity = remote_control_actual.GetNaturalGravity();
                if (align_target_valid)
                {
                    planeNrml = ((m_gps_crds - align_gps_coords));
                }
                if (!align_target_valid)
                {
                    planeNrml = gravity;
                }

                planeNrml.Normalize();
                Vector3D perpendicularVector = Vector3D.CalculatePerpendicularVector(planeNrml);
                perpendicularVector.Normalize();
                Vector3D centerPoint = m_gps_crds;
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
                if (gridDataList.Count > 0 && init_grid_complete)
                {
                    created_grid = true;
                    pb_i_act.CustomData = "";
                    can_init = false;
                    i_init = false;
                    it_ag = "";
                }

                total_mining_runs = gridDataList.Count;

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
                Echo($"{drone_messages_list.Count} {droneDataList.Count}");
                //process drone message list here
                if (drone_messages_list.Count < droneDataList.Count)
                {
                    Drone_Message = true;
                }
                Echo($"{drone_messages_list.Count} {droneDataList.Count}");

                if (drone_messages_list.Count > 0)
                {
                    //pull first message in the list if valid
                    data_in_drone = drone_messages_list[0].Data.ToString();
                    Get_Drone_Message_Data(data_in_drone);                    
                    recievedMessagetoDroneDatabase(recievedDroneName);                    
                }
                Echo($"{drone_messages_list.Count} {droneDataList.Count}");
                if (drone_messages_list.Count > droneDataList.Count)
                {
                    Drone_Message = false;
                }
                Echo($"{drone_messages_list.Count} {droneDataList.Count}");

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

                if (droneDataList.Count == 0 && !pinged || droneDataList.Count > 0 && !pinged)
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
            int startInstructions = Runtime.CurrentInstructionCount;
            
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
            Echo($"ProcessInterface: {Runtime.CurrentInstructionCount - startInstructions}");
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
            if (display_tag_drone.Count == 0 || droneDataList.Count == 0 || display_tag_drone[0] == null) return;

            if (renew_header)
            {
                droneInformation.Clear().Append($"Mining Drone Status - GMDC {ver}\n");
                renew_header = false;
            }

            int dronesPerDisplay = drones_per_screen * display_tag_drone.Count;
            if (dronesPerDisplay < droneDataList.Count)
            {
                Echo($"Insufficient displays '{dp_drn_tag}': {dronesPerDisplay} < {droneDataList.Count}");
                return;
            }

            for (int i = 0; i < droneDataList.Count; i += 2)
            {
                bool hasPair = i + 1 < droneDataList.Count;
                scrnbldr(i, hasPair ? i + 1 : i, hasPair);

                int displayIndex = i / drones_per_screen;
                if (displayIndex < display_tag_drone.Count && display_tag_drone[displayIndex] != null &&
                    (i % drones_per_screen == drones_per_screen - 2 || i >= droneDataList.Count - 2))
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
            bores_completed = CntTrueVlsGridBoreDataFinished(gridDataList);
            bores_remaining = total_mining_runs - bores_completed;
            total_drones_mining = CntTrueVlsdroneDataBool(droneDataList,d => d.droneMining);
            total_drones_undocking = CntIntVlsDroneDataInt(droneDataList,d =>d.droneControlSequence, 2);

            DroneStats stats = new DroneStats();
            for (int i = 0; i < droneDataList.Count; i++)
            {
                
                string status = droneDataList[i].droneControlStatus;
                string damage = droneDataList[i].droneDamageState;
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

            if (gridDataList.Count > 0)
            {
                if (boreQueueCounts.Length < gridDataList.Count)
                    boreQueueCounts = new int[gridDataList.Count];
                Array.Clear(boreQueueCounts, 0, gridDataList.Count);

                for (int d = 0; d < droneDataList.Count; d++)
                    if (droneDataList[d].droneGPSListPosition >= 0 && droneDataList[d].droneGPSListPosition < boreQueueCounts.Length)
                    {
                        boreQueueCounts[droneDataList[d].droneGPSListPosition]++;
                    }
                        

                for (int l = 0; l < gridDataList.Count; l++)
                    if (gridDataList[l].occupied && boreQueueCounts[l] == 0)
                    {
                        gridData gridnew = gridDataList[l];
                        gridnew.occupied = false;
                        gridDataList[l] = gridnew;
                    }
            }
            Echo($"DroneUndockCheck: {Runtime.CurrentInstructionCount - startInstructions}");
        }

        public void updateDisplay(int i)
        {
            dp_txm.Append($"Drone Controller Status - GMDC {ver} - [{drone_tag}] {icon}");
            dp_txm.Append('\n');
            if (droneDataList[i].droneControlSequence == 12)
            {
                dp_txm.Append('\n');
                dp_txm.Append("Mining seq. complete"); ;
            }

            if (gridDataList.Count > 0)
            {
                dp_txm.Append('\n');
                dp_txm.Append("Grid pos: " + gridDataList.Count);
                dp_txm.Append('\n');
                dp_txm.Append("Grid dist: " + cst_dt7 + "m #X: " + cst_dt8 + " #Y: " + cst_dt9);
                dp_txm.Append('\n');
                dp_txm.Append("Grid OK: " + mining_grid_valid);
                dp_txm.Append('\n');
                dp_txm.Append("Bores: " + gridDataList.Count + " Remain: " + bores_remaining + "  Skip: " + skip_bores_number);
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
                dp_txm.Append('\n');
                dp_txm.Append("Mine seq. complete");
                dp_txm.Append('\n');
            }
        }
        void Get_Drone_Message_Data(string data_message)
        {
            
            // get custom data from programmable block
            String[] msgdta = data_message.Split(':');
            if (msgdta.Length < 22) 
            {
                Echo($"Recieved bad message");
                return;
            }

            //Define GPS coordinates from 
            if (msgdta.Length > 5)
            {
                recievedDroneName = msgdta[0];
                if (recievedDroneName.Contains(drone_tag))
                {
                    recievedDroneDamageState = msgdta[1];
                    recievedDroneTunnelFinished = msgdta[2];
                    receivedDroneControlStatus = msgdta[3];
                    receievdDroneDockStatus = msgdta[4];
                    recievedDroneUndockStatus = msgdta[5];
                    recievedDroneAutopilot = msgdta[6];
                    recievedDroneAutopilotEnabled = msgdta[7];
                    recievedDroneLocationX = msgdta[8];
                    recievedDroneLocationY = msgdta[9];
                    recievedDroneLocationZ = msgdta[10];
                    receievdDroneDrillDepth = msgdta[11];
                    receivedDroneMinedDistance = msgdta[12];
                    recivedDroneMineStart = msgdta[13];
                    recivedDroneCharge = msgdta[14];
                    recievedDroneGas = msgdta[15];
                    recivedDroneOre = msgdta[16];
                    if (msgdta.Length > 16)
                    {
                        recievedDroneListPosition = msgdta[17];
                    }
                    if (msgdta.Length > 17)
                    {
                        recivedDroneCargoFull = msgdta[18];
                    }
                    if (msgdta.Length > 18)
                    {
                        recievedDroneRechargeRequest = msgdta[19];
                    }
                    if (msgdta.Length > 19)
                    {
                        recievedDroneAutoDockEnabled = msgdta[20];
                    }
                    if (msgdta.Length > 20)
                    {
                        recievedDroneDockingReady = msgdta[21];
                    }
                }
                else
                {
                    recievedDroneName = "";
                    recievedDroneDamageState = "";
                    recievedDroneTunnelFinished = "";
                    receivedDroneControlStatus = "";
                    receievdDroneDockStatus = "";
                    recievedDroneUndockStatus = "";
                    recievedDroneAutopilot = "";
                    recievedDroneAutopilotEnabled = "";
                    recievedDroneLocationX = "";
                    recievedDroneLocationY = "";
                    recievedDroneLocationZ = "";
                    receievdDroneDrillDepth = "";
                    receivedDroneMinedDistance = "";
                    recivedDroneMineStart = "";
                    recivedDroneCharge = "";
                    recievedDroneGas = "";
                    recivedDroneOre = "";
                    recievedDroneListPosition = "";
                    recivedDroneCargoFull = "";
                    recievedDroneRechargeRequest = "";
                    recievedDroneAutoDockEnabled = "";
                    recievedDroneDockingReady = "";

                }
                //parsing in wrong area
                if (recievedDroneListPosition == "")
                {
                    _recievedDroneListPosition = -1;
                }
                else
                {
                    if (!int.TryParse(recievedDroneListPosition, out _recievedDroneListPosition))
                    {
                        _recievedDroneListPosition = -1;
                    }
                }
                if (recivedDroneCharge == "")
                {
                    rc_d_cn = 0.0;
                }
                if (!double.TryParse(recivedDroneCharge, out rc_d_cn))
                {
                    rc_d_cn = 0.0;
                }
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
                if (!double.TryParse(rm_cst_dat9, out align_gps_coords.X))
                {
                    align_gps_coords.X = 0.0;
                    rm_cst_dat9 = "";
                }
                if (!double.TryParse(rm_cst_dat10, out align_gps_coords.Y))
                {
                    align_gps_coords.Y = 0.0;
                    rm_cst_dat10 = "";
                }
                if (!double.TryParse(rm_cst_dat11, out align_gps_coords.Z))
                {
                    align_gps_coords.Z = 0.0;
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
                if (!double.TryParse(cst_dt2, out m_gps_crds.X))
                {
                    m_gps_crds.X = 0.0;
                    cst_dt2 = "";
                }
                if (!double.TryParse(cst_dt3, out m_gps_crds.Y))
                {
                    m_gps_crds.Y = 0.0;
                    cst_dt3 = "";
                }
                if (!double.TryParse(cst_dt4, out m_gps_crds.Z))
                {
                    m_gps_crds.Z = 0.0;
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
            cl[0] = $"{droneDataList[ivl].droneName} Status: {droneDataList[ivl].droneDamageState} {droneDataList[ivl].droneControlStatus}";
            cl[1] = $"{droneDataList[ivl].droneName} Docked: {droneDataList[ivl].droneDockStatus}";
            cl[2] = $"{droneDataList[ivl].droneName} Undocked: {droneDataList[ivl].droneUndockStatus}";
            cl[3] = $"{droneDataList[ivl].droneName} Finished: {droneDataList[ivl].droneTunnelFinished}";
            cl[4] = $"{droneDataList[ivl].droneName} Mining: {droneDataList[ivl].droneMining}";
            cl[5] = $"{droneDataList[ivl].droneName} Waiting: {droneDataList[ivl].droneMustWait} Reset: {droneDataList[ivl].droneResetFunc}";
            cl[6] = $"Charge: {droneDataList[ivl].droneStoredCharge}% Tank: {droneDataList[ivl].droneStoredGas}% Cargo: {droneDataList[ivl].droneStoredOre}%";
            cl[7] = $"Drill depth: {droneDataList[ivl].droneDrillDepth}m Start: {droneDataList[ivl].droneMineDistanceStart}m";
            cl[8] = $"Current depth: {droneDataList[ivl].droneMinedDistanceCurrent}m";
            cl[9] = $"Drone control seq: {droneDataList[ivl].droneControlSequence} Recall seq: {droneDataList[ivl].droneRecallSequence} {droneDataList[ivl].droneRecallList}";
            cl[10] = $"Location: {droneDataList[ivl].droneGPSListPosition} Asnd: {droneDataList[ivl].droneAssignedCoordinates} Unit OK: {dst[ivl]}";
            cl[11] = $"X: {droneDataList[ivl].droneLocationX} Y: {droneDataList[ivl].droneLocationY} Z: {droneDataList[ivl].droneLocationZ}";

            if (slu)
            {
                cl2[0] = $"{droneDataList[ivl2].droneName} Status: {droneDataList[ivl2].droneDamageState} {droneDataList[ivl2].droneControlStatus}";
                cl2[1] = $"{droneDataList[ivl2].droneName} Docked: {droneDataList[ivl2].droneDockStatus}";
                cl2[2] = $"{droneDataList[ivl2].droneName} Undocked: {droneDataList[ivl2].droneUndockStatus}";
                cl2[3] = $"{droneDataList[ivl2].droneName} Finished: {droneDataList[ivl2].droneTunnelFinished}";
                cl2[4] = $"{droneDataList[ivl2].droneName} Mining: {droneDataList[ivl2].droneMining}";
                cl2[5] = $"{droneDataList[ivl2].droneName} Waiting: {droneDataList[ivl2].droneMustWait} Reset: {droneDataList[ivl2].droneResetFunc}";
                cl2[6] = $"Charge: {droneDataList[ivl2].droneStoredCharge}% Tank: {droneDataList[ivl2].droneStoredGas}% Cargo: {droneDataList[ivl2].droneStoredOre}%";
                cl2[7] = $"Drill depth: {droneDataList[ivl2].droneDrillDepth}m Start: {droneDataList[ivl2].droneMineDistanceStart}m";
                cl2[8] = $"Current depth: {droneDataList[ivl2].droneMinedDistanceCurrent}m";
                cl2[9] = $"Drone control seq: {droneDataList[ivl2].droneControlSequence} Recall seq: {droneDataList[ivl2].droneRecallSequence} {droneDataList[ivl2].droneRecallList}";
                cl2[10] = $"Location: {droneDataList[ivl2].droneGPSListPosition} Asnd: {droneDataList[ivl2].droneAssignedCoordinates} Unit OK: {dst[ivl]}";
                cl2[11] = $"X: {droneDataList[ivl2].droneLocationX} Y: {droneDataList[ivl2].droneLocationY} Z: {droneDataList[ivl2].droneLocationZ}";
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

            for (int i = 0; i < gridDataList.Count; i++)
            {
                gridData gridnew = gridDataList[i];
                
                for (int j = 0; j < droneDataList.Count; j++)
                {
                    droneData droneInfo = droneDataList[j];
                    if (!gridDataList[i].occupied)
                    {
                        drone_namer = "";
                    }
                    else if (i == droneDataList[j].droneGPSListPosition)
                    {
                        
                        drone_namer = droneDataList[j].droneName;
                        droneInfo.droneAssignsCount++;
                        droneDataList[j] = droneInfo;
                    }
                    if (droneDataList[j].droneGPSListPosition > 1)
                    {
                        gridnew.occupied = false;
                        gridDataList[i] = gridnew;
                        
                    }
                    droneInfo.droneAssignsCount = 0;
                    droneDataList[j] = droneInfo;

                }
                if (!gridDataList[i].finished)
                {
                    dp_txl.Append('\n');
                    dp_txl.Append($"i: {i}  Mining: {gridDataList[i].occupied.ToString()}  Finished: {gridDataList[i].finished.ToString()} Drone: {drone_namer}");

                }
                if (i == gridDataList.Count - 1)
                {
                    listgenerator_finished = true;
                }
                percent_list = ((double)i / (double)gridDataList.Count) * 100;
                yield return false;
            }
            yield return true;
        }

        IEnumerator<bool> GenGrdPosits(Vector3D centerPoint, Vector3D planeNormal, double gridSize, int numPointsX, int numPointsY, bool coreout)
        {
            //debugcount++;
            //initgridcount++;
            //List<Vector3D> grdPositins = new List<Vector3D>();
            gridData gridnew = new gridData();
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
                    gridnew.gps_coords = position;
                    gridnew.occupied = false;
                    gridnew.finished = false;
                    gridDataList.Add(gridnew);

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
                            gridnew.gps_coords = position;
                            gridnew.occupied = false;
                            gridnew.finished = false;
                            gridDataList.Add(gridnew);

                        }
                        yield return false;
                    }
                }
            }
            gridcount = gridcount_inner + gridcount_outer;
            percent_grid = (double)gridDataList.Count / (double)gridcount;
            
            if (gridDataList.Count == gridcount)
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

            if (gridDataList.Count > 0)
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

                for (int i = 0; i < gridDataList.Count; i++)
                {
                    sprite_total++;
                    Vector3D relativePoint = gridDataList[i].gps_coords - centerPoint;
                    double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                    double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                    var CentX = (float)xPlanar;
                    var CentY = (float)yPlanar;
                    string Image;
                    var bore_colour = new Color();
                    var alpha_bytes = 1.0f;
                    Image = gridDataList[i].finished ? "CircleHollow" : "Circle";
                    alpha_bytes = gridDataList[i].occupied ? 1.0f : 0.5f;
                    bore_colour = gridDataList[i].occupied ? Color.LightSkyBlue : Color.DeepSkyBlue;

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
                    
                    sprites.Add(sprite);
                    percent_list_vis = (((double)i + (double)1) / ((double)gridDataList.Count)) * 100;
                    spritecount++;
                    yield return false;
                }

                if (droneDataList.Count > 0)
                {
                    drone_total = 0;
                    for (int i = 0; i < droneDataList.Count; i++)
                    {
                        double drone_locale_x = 0.0;
                        double drone_locale_y = 0.0;
                        double drone_locale_z = 0.0;
                        drone_total++;

                            drone_locale_x = droneDataList[i].droneLocationX;

                            drone_locale_y = droneDataList[i].droneLocationY;

                            drone_locale_z = droneDataList[i].droneLocationZ;
                        

                        Vector3D Drone_point = new Vector3D(drone_locale_x, drone_locale_y, drone_locale_z);

                        Vector3D relativePoint = Drone_point - centerPoint;
                        double xPlanar = Vector3D.Dot(relativePoint, xAxis);
                        double yPlanar = Vector3D.Dot(relativePoint, yAxis);
                        var CentX = (float)xPlanar;
                        var CentY = (float)yPlanar;
                        string Image_drone = "";
                        var bore_colour_drone = new Color();
                        var alpha_val = 1.0f;

                        if (droneDataList[i].droneControlStatus.Contains("Docked") || droneDataList[i].droneControlStatus.Contains("Undocked") || droneDataList[i].droneControlStatus.Contains("Docking") || droneDataList[i].droneControlStatus.Contains("Undocking"))
                        {
                            alpha_val = 0.25f;
                        }
                        else
                        {
                            alpha_val = 1.0f;
                        }
                        if (!droneDataList[i].droneMining)
                        {
                            Image_drone = "Circle";
                            bore_colour_drone = Color.Gray;
                        }
                        if (droneDataList[i].droneMining)
                        {
                            Image_drone = "Circle";

                            if (droneDataList[i].droneControlStatus.Contains("Min"))
                            {
                                bore_colour_drone = Color.Purple;
                            }
                            else if (droneDataList[i].droneControlStatus.Contains("Exit"))
                            {
                                bore_colour_drone = Color.Orange;
                            }
                            else if (droneDataList[i].droneControlStatus.Contains("RTB: Ready"))
                            {
                                bore_colour_drone = Color.Green;
                            }
                            else if (droneDataList[i].droneControlStatus.Contains("Undock"))
                            {
                                bore_colour_drone = Color.Yellow;
                            }
                            else
                            {
                                bore_colour_drone = Color.Navy;
                            }
                            alpha_val = 1.0f;
                        }
                        if (droneDataList[i].droneDamageState == "DMG")
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
                        if (droneDataList[i].droneCargoFull || droneDataList[i].droneRechargeRequest)
                        {
                            
                            if (droneDataList[i].droneRechargeRequest && droneDataList[i].droneCargoFull)
                            {
                                bore_colour_drone = Color.White;
                            }
                            else if (droneDataList[i].droneRechargeRequest)
                            {
                                bore_colour_drone = Color.YellowGreen;
                            }
                            else if (droneDataList[i].droneCargoFull)
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

                        if (droneDataList[i].droneControlStatus.Contains("Recharg") || droneDataList[i].droneControlStatus.Contains("Unload") || droneDataList[i].droneRechargeRequest)
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
                            Data = $"{droneDataList[i].droneName}- ({droneDataList[i].droneStoredCharge}%)",
                            Position = position,
                            RotationOrScale = 0.3f,
                            Size = sizer * 0.5f,
                            Color = bore_colour_drone.Alpha(alpha_val),
                            Alignment = TextAlignment.CENTER,
                            FontId = "White"
                        };
                        sprites.Add(sprite_name);
                        percent_list_drones = ((double)drone_total / (double)droneDataList.Count) * 100;
                        spritecount++;
                        yield return false;
                    }
                }
                if (droneDataList.Count == 0)
                {
                    if (sprite_total == gridDataList.Count)
                    {
                        frame_generator_finished = true;
                    }

                    else
                    {
                        frame_generator_finished = false;
                    }
                }
                if (droneDataList.Count > 0)
                {
                    if (sprite_total == gridDataList.Count && drone_total == droneDataList.Count)
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
            current_gps_idx = (current_gps_idx + 1) % gridDataList.Count;
            next_gps_crds = gridDataList[current_gps_idx].gps_coords;
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
        int CntTrueVlsdroneDataBool(List<droneData> list, Func<droneData, bool> fieldSelector)
        {
            int truCnt = 0;
            foreach (droneData drone in list)
            {
                if (fieldSelector(drone))
                {
                    truCnt++;
                }
            }
            return truCnt;
        }
        int CntTrueVlsGridBoreDataFinished(List<gridData> boreList)
        {
            int count = 0;
            for (int i = 0; i < boreList.Count; i++)
            {
                if (boreList[i].finished) count++;
            }
            return count;
        }
        int CntIntVlsDroneDataInt(List<droneData> list, Func<droneData, int> fieldSelector, int matchValue)
        {
            int truCnt = 0;

            foreach (droneData drone in list)
            {
                if (fieldSelector(drone) == matchValue)
                {
                    truCnt++;
                }
            }
            return truCnt;
        }
        int CntStsVlstDroneData(List<droneData> list, Func<droneData, string> fieldSelector, string matchValue)
        {
            int trueCount = 0;

            foreach (droneData drone in list)
            {
                if (fieldSelector(drone).Contains(matchValue))
                {
                    trueCount++;
                }
            }
            return trueCount;
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


        public void drone_command_builder(string cdata_1, string xpos, string ypos, string zpos, string cdata_5, string cmdo, string data_6, string idepth, string xpos2, string ypos2, string zpos2)
        {
            const string baseFormat = "GPS:{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:";
            const string astFormat = "GPS:PAD:{0}:{1}:{2}:#FF75C9F1:";

            c.Clear().EnsureCapacity(align_target_valid ? 120 : 80); // ~80 chars base, ~40 more if asteroid
            c.AppendFormat(baseFormat, cdata_1, xpos, ypos, zpos, cdata_5, cmdo, data_6, idepth);
            if (align_target_valid) c.AppendFormat(astFormat, xpos2, ypos2, zpos2);
        }
        void transmit_to_drone(int di)
        {
            IGC.SendBroadcastMessage(tx_chan, droneDataList[di].droneTransmissionOutput, TransmissionDistance.TransmissionDistanceMax);
        }
        void GetStoredData()
        {
            gridData gridnew = new gridData();
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
                        gridnew.finished = (bnParsed && bn > 0);
                        gridnew.occupied = (bcParsed && bc > 0);


                        if (str_datai.Length >= 5) // Full bn:bc:x:y:z
                        {
                            double x = double.TryParse(str_datai[2], out bx) ? bx : 0.0;
                            double y = double.TryParse(str_datai[3], out by) ? by : 0.0;
                            double z = double.TryParse(str_datai[4], out bz) ? bz : 0.0;
                            gridnew.gps_coords = new Vector3D(x, y, z);

                            gridDataList.Add(gridnew);                        }
                        else
                        {
                            gridnew.finished = false;
                            gridnew.occupied = false;
                            gridnew.gps_coords = new Vector3D(0, 0, 0);
                            gridDataList.Add(gridnew); // Default position if incomplete
                        }
                    }
                }
            }
        }
        void reset_drone_data()
        {


            dcs.Clear();
            dst.Clear();

            droneDataList.Clear();
        }

        void reset_drone_list()
        {


            dcs = new List<double>();
            dst = new List<bool>();

            //replacement with below
            droneDataList = new List<droneData>();
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
            
            sprites = new List<MySprite>();
            rm_ctl_all = new List<IMyRemoteControl>();
            rm_ctl_tag = new List<IMyRemoteControl>();
            droneDataList = new List<droneData>();
            gridDataList = new List<gridData>();
            bores_regen = false;
            cl = new List<string>();
            cl2 = new List<string>();
            tla = new List<int>();
            rst = new List<int>();
            fct = new List<string>();
            dcs = new List<double>();
            dst = new List<bool>();
            miningCoordinatesNew = new StringBuilder();
            dp_txm = new StringBuilder();
            dp_txl = new StringBuilder();
            droneInformation = new StringBuilder();
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

        struct droneData
        {
            public string droneName; //0
            public string droneDamageState; //1
            public string droneControlStatus; //2
            public bool droneTunnelFinished;  //3          
            public bool droneDockStatus; //4
            public bool droneUndockStatus;//5
            public bool droneAutopilotStatus;//6
            public bool droneAutopilotEnabled;//7
            public double droneLocationX; //8
            public double droneLocationY; //9
            public double droneLocationZ; //10
            public double droneDrillDepth;//11
            public double droneMinedDistanceCurrent; //12
            public double droneMineDistanceStart; //13                        
            public double droneStoredCharge; //14
            public double droneStoredGas;//15
            public double droneStoredOre;//16
            public int droneGPSListPosition; //17
            public bool droneCargoFull;//18
            public bool droneRechargeRequest;//19
            public bool droneAutodockEnabled; //20
            public bool droneDockingReady; //21  
            public Vector3D droneGPSCoordinatesDS;
            public bool droneReady;
            public bool droneMining;           
            public bool droneMustWait;
            public int droneControlSequence;
            public int droneRecallSequence;
            public bool droneRecallList;
            public bool droneResetFunc;
            public string droneTransmissionOutput;
            public bool droneTransmissionStatus;                       
            public int droneAssignsCount;
            public bool droneAssignedCoordinates;

            public droneData (string droneName, string droneDamageState, string droneControlStatus, bool droneTunnelFinished, bool droneUndockStatus, bool droneDockStatus, bool droneAutopilotStatus, bool droneAutoPilotEnabled,
                double droneLocationX, double droneLocationY, double droneLocationZ, double droneDrillDepth, double droneMinedDistanceCurrent, double droneMineDistanceStart, double droneStoredCharge, double droneStoredGas, double droneStoredOre, 
                int droneGPSListPosition, bool droneCargoFull, bool droneRechargeRequest, bool droneAutodockEnabled, bool droneDockingReady, 
                Vector3D droneGPSCoordinatesDS, bool droneReady, bool droneMining, bool droneMustWait, int droneControlSequence, 
                int droneRecallSequence, bool droneRecallList, bool droneResetFunc, 
                string droneTransmissionOutput, bool droneTransmissionStatus, int droneAssignsCount, 
                bool droneAssignedCoordinates)
            {
                this.droneName = droneName;
                this.droneDamageState = droneDamageState;
                this.droneDamageState = droneDamageState;
                this.droneControlStatus = droneControlStatus;
                this.droneTunnelFinished = droneTunnelFinished;
                this.droneDockStatus = droneDockStatus;
                this.droneUndockStatus = droneUndockStatus;
                this.droneAutopilotStatus = droneAutopilotStatus;
                this.droneAutopilotEnabled = droneAutoPilotEnabled;
                this.droneLocationX = droneLocationX;
                this.droneLocationY = droneLocationY;
                this.droneLocationZ = droneLocationZ;
                this.droneDrillDepth = droneDrillDepth;
                this.droneMinedDistanceCurrent = droneMinedDistanceCurrent;
                this.droneMineDistanceStart = droneMineDistanceStart;
                this.droneStoredCharge = droneStoredCharge;
                this.droneStoredGas = droneStoredGas;
                this.droneStoredOre = droneStoredOre;
                this.droneGPSListPosition = droneGPSListPosition;
                this.droneCargoFull = droneCargoFull;
                this.droneRechargeRequest = droneRechargeRequest;
                this.droneAutodockEnabled = droneAutodockEnabled;
                this.droneDockingReady = droneDockingReady;
                this.droneGPSCoordinatesDS = droneGPSCoordinatesDS;
                this.droneReady = droneReady;
                this.droneMining = droneMining;
                this.droneMustWait = droneMustWait;
                this.droneControlSequence = droneControlSequence;
                this.droneRecallSequence = droneRecallSequence;
                this.droneRecallList = droneRecallList;
                this.droneResetFunc = droneResetFunc;
                this.droneTransmissionOutput = droneTransmissionOutput;
                this.droneTransmissionStatus = droneTransmissionStatus;
                this.droneAssignsCount = droneAssignsCount;
                this.droneAssignedCoordinates = droneAssignedCoordinates;
            }
        }

        public void recievedMessagetoDroneDatabase(string recievedDroneName)
        {
            
            #region drone_message_data_processing            
            if (!string.IsNullOrEmpty(recievedDroneName))
            {                
                found = false;
                //drone does not exist assume none and add first
                if (droneDataList.Count <= 0)
                {
                    droneData droneInfo = new droneData();
                    droneInfo.droneName = recievedDroneName;
                    droneInfo.droneDamageState = recievedDroneDamageState;
                    if(!bool.TryParse(recievedDroneTunnelFinished, out droneInfo.droneTunnelFinished))
                    {
                        droneInfo.droneTunnelFinished = false;
                    }                    
                    droneInfo.droneControlStatus = receivedDroneControlStatus;
                    if(!bool.TryParse(receievdDroneDockStatus, out droneInfo.droneDockStatus))
                    {
                        droneInfo.droneDockStatus = false;
                    }
                    if (!bool.TryParse(recievedDroneUndockStatus, out droneInfo.droneUndockStatus))
                    {
                        droneInfo.droneUndockStatus = false;
                    }
                    if(!bool.TryParse(recievedDroneAutopilot, out droneInfo.droneAutopilotStatus))
                    {
                        droneInfo.droneAutopilotStatus = false;
                    }
                    if (!bool.TryParse(recievedDroneAutopilotEnabled, out droneInfo.droneAutopilotEnabled))
                    {
                        droneInfo.droneAutopilotEnabled = false;
                    }
                    if (!double.TryParse(receievdDroneDrillDepth, out droneInfo.droneDrillDepth))
                    {
                        droneInfo.droneDrillDepth = 0.0;
                    }
                    if (!double.TryParse(receivedDroneMinedDistance, out droneInfo.droneMinedDistanceCurrent))
                    {
                        droneInfo.droneMinedDistanceCurrent = 0.0;
                    }
                    if (!double.TryParse(recivedDroneMineStart, out droneInfo.droneMineDistanceStart))
                    {
                        droneInfo.droneMineDistanceStart = 0.0;
                    }
                    droneInfo.droneGPSListPosition = -1;
                    if(!double.TryParse(recievedDroneLocationX, out droneInfo.droneLocationX))
                    {
                        droneInfo.droneLocationX = 0.0;
                    }
                    if (!double.TryParse(recievedDroneLocationY, out droneInfo.droneLocationY))
                    {
                        droneInfo.droneLocationY = 0.0;
                    }
                    if (!double.TryParse(recievedDroneLocationZ, out droneInfo.droneLocationZ))
                    {
                        droneInfo.droneLocationZ = 0.0;
                    }
                    if (!double.TryParse(recivedDroneCharge, out droneInfo.droneStoredCharge))
                    {
                        droneInfo.droneStoredCharge = 0.0;
                    }
                    if (!double.TryParse(recievedDroneGas, out droneInfo.droneStoredGas))
                    {
                        droneInfo.droneStoredGas = 0.0;
                    }
                    if (!double.TryParse(recivedDroneOre, out droneInfo.droneStoredOre))
                    {
                        droneInfo.droneStoredOre = 0.0;
                    }
                    if (!bool.TryParse(recivedDroneCargoFull, out droneInfo.droneCargoFull))
                    {
                        droneInfo.droneCargoFull = false;
                    }
                    if (!bool.TryParse(recievedDroneRechargeRequest, out droneInfo.droneRechargeRequest))
                    {
                        droneInfo.droneRechargeRequest = false;
                    }
                    if(!bool.TryParse(recievedDroneAutoDockEnabled, out droneInfo.droneAutodockEnabled))
                    {
                        droneInfo.droneAutodockEnabled = false;
                    }
                    if (!bool.TryParse(recievedDroneDockingReady, out droneInfo.droneDockingReady))
                    {
                        droneInfo.droneDockingReady = false;
                    }
                    droneInfo.droneGPSCoordinatesDS = remote_control_actual.GetPosition();
                    droneInfo.droneReady = false;
                    droneInfo.droneMining = false;
                    droneInfo.droneMustWait = true;
                    droneInfo.droneControlSequence = 0;
                    droneInfo.droneRecallSequence = 0;
                    droneInfo.droneRecallList = false;
                    droneInfo.droneResetFunc = false;
                    droneInfo.droneTransmissionOutput = "";
                    droneInfo.droneTransmissionStatus = true;
                    droneInfo.droneAssignsCount = 0;
                    droneInfo.droneAssignedCoordinates = false;                   
                    dcs.Add(0.0);
                    dst.Add(true);
                    droneDataList.Add(droneInfo);
                    Confirmed_Drone_Message = true;
                    recievedDroneNameIndex = 0;
                }
                //drones do exist so check current drone list to see if it exists
                if (droneDataList.Count > 0)
                {                    

                    for (int i = 0; i < droneDataList.Count; i++)
                    {
                        found = false;                        
                        string nametag = droneDataList[i].droneName;
                        
                        //drone name found so update data
                        if (nametag == recievedDroneName)
                        {
                            droneData droneInfo = new droneData();
                            droneInfo = droneDataList[i];
                            found = true;
                            if (!bool.TryParse(recievedDroneTunnelFinished, out droneInfo.droneTunnelFinished))
                            {
                                droneInfo.droneTunnelFinished = false;
                            }
                            droneInfo.droneControlStatus = receivedDroneControlStatus;
                            if (!bool.TryParse(receievdDroneDockStatus, out droneInfo.droneDockStatus))
                            {
                                droneInfo.droneDockStatus = false;
                            }
                            if (!bool.TryParse(recievedDroneUndockStatus, out droneInfo.droneUndockStatus))
                            {
                                droneInfo.droneUndockStatus = false;
                            }
                            if (!bool.TryParse(recievedDroneAutopilot, out droneInfo.droneAutopilotStatus))
                            {
                                droneInfo.droneAutopilotStatus = false;
                            }
                            if (!bool.TryParse(recievedDroneAutopilotEnabled, out droneInfo.droneAutopilotEnabled))
                            {
                                droneInfo.droneAutopilotEnabled = false;
                            }
                            if (!double.TryParse(receievdDroneDrillDepth, out droneInfo.droneDrillDepth))
                            {
                                droneInfo.droneDrillDepth = 0.0;
                            }
                            if (!double.TryParse(receivedDroneMinedDistance, out droneInfo.droneMinedDistanceCurrent))
                            {
                                droneInfo.droneMinedDistanceCurrent = 0.0;
                            }
                            if (!double.TryParse(recivedDroneMineStart, out droneInfo.droneMineDistanceStart))
                            {
                                droneInfo.droneMineDistanceStart = 0.0;
                            }
                            droneInfo.droneGPSListPosition = _recievedDroneListPosition;
                            if (!double.TryParse(recievedDroneLocationX, out droneInfo.droneLocationX))
                            {
                                droneInfo.droneLocationX = 0.0;
                            }
                            if (!double.TryParse(recievedDroneLocationY, out droneInfo.droneLocationY))
                            {
                                droneInfo.droneLocationY = 0.0;
                            }
                            if (!double.TryParse(recievedDroneLocationZ, out droneInfo.droneLocationZ))
                            {
                                droneInfo.droneLocationZ = 0.0;
                            }
                            if (!double.TryParse(recivedDroneCharge, out droneInfo.droneStoredCharge))
                            {
                                droneInfo.droneStoredCharge = 0.0;
                            }
                            if (!double.TryParse(recievedDroneGas, out droneInfo.droneStoredGas))
                            {
                                droneInfo.droneStoredGas = 0.0;
                            }
                            if (!double.TryParse(recivedDroneOre, out droneInfo.droneStoredOre))
                            {
                                droneInfo.droneStoredOre = 0.0;
                            }
                            if (!bool.TryParse(recivedDroneCargoFull, out droneInfo.droneCargoFull))
                            {
                                droneInfo.droneCargoFull = false;
                            }
                            if (!bool.TryParse(recievedDroneRechargeRequest, out droneInfo.droneRechargeRequest))
                            {
                                droneInfo.droneRechargeRequest = false;
                            }
                            if (!bool.TryParse(recievedDroneAutoDockEnabled, out droneInfo.droneAutodockEnabled))
                            {
                                droneInfo.droneAutodockEnabled = false;
                            }
                            if (!bool.TryParse(recievedDroneDockingReady, out droneInfo.droneDockingReady))
                            {
                                droneInfo.droneDockingReady = false;
                            }
                            droneInfo.droneTransmissionStatus = true;
                            
                            if (i < dcs.Count)
                            {
                                dcs[i] = rc_d_cn;
                                
                                if (dcs[i] <= bclm)
                                {
                                    dst[i] = false;
                                }                                
                                if (dcs[i] > bclm)
                                {
                                    dst[i] = true;
                                }
                            }                            
                            droneDataList[i] = droneInfo;
                            Confirmed_Drone_Message = true;
                            recievedDroneNameIndex = i;                            
                            break;
                        }
                        //drone not found at end of list so add drone to list
                        if (i == (droneDataList.Count - 1) && !nametag.Equals(recievedDroneName) && !found)
                        {
                            droneData droneInfo = new droneData();
                            droneInfo = droneDataList[i];
                            droneInfo.droneName = recievedDroneName;
                            droneInfo.droneDamageState = recievedDroneDamageState;
                            if (!bool.TryParse(recievedDroneTunnelFinished, out droneInfo.droneTunnelFinished))
                            {
                                droneInfo.droneTunnelFinished = false;
                            }
                            droneInfo.droneControlStatus = receivedDroneControlStatus;
                            if (!bool.TryParse(receievdDroneDockStatus, out droneInfo.droneDockStatus))
                            {
                                droneInfo.droneDockStatus = false;
                            }
                            if (!bool.TryParse(recievedDroneUndockStatus, out droneInfo.droneUndockStatus))
                            {
                                droneInfo.droneUndockStatus = false;
                            }
                            if (!bool.TryParse(recievedDroneAutopilot, out droneInfo.droneAutopilotStatus))
                            {
                                droneInfo.droneAutopilotStatus = false;
                            }
                            if (!bool.TryParse(recievedDroneAutopilotEnabled, out droneInfo.droneAutopilotEnabled))
                            {
                                droneInfo.droneAutopilotEnabled = false;
                            }
                            if (!double.TryParse(receievdDroneDrillDepth, out droneInfo.droneDrillDepth))
                            {
                                droneInfo.droneDrillDepth = 0.0;
                            }
                            if (!double.TryParse(receivedDroneMinedDistance, out droneInfo.droneMinedDistanceCurrent))
                            {
                                droneInfo.droneMinedDistanceCurrent = 0.0;
                            }
                            if (!double.TryParse(recivedDroneMineStart, out droneInfo.droneMineDistanceStart))
                            {
                                droneInfo.droneMineDistanceStart = 0.0;
                            }
                            droneInfo.droneGPSListPosition = -1;
                            if (!double.TryParse(recievedDroneLocationX, out droneInfo.droneLocationX))
                            {
                                droneInfo.droneLocationX = 0.0;
                            }
                            if (!double.TryParse(recievedDroneLocationY, out droneInfo.droneLocationY))
                            {
                                droneInfo.droneLocationY = 0.0;
                            }
                            if (!double.TryParse(recievedDroneLocationZ, out droneInfo.droneLocationZ))
                            {
                                droneInfo.droneLocationZ = 0.0;
                            }
                            if (!double.TryParse(recivedDroneCharge, out droneInfo.droneStoredCharge))
                            {
                                droneInfo.droneStoredCharge = 0.0;
                            }
                            if (!double.TryParse(recievedDroneGas, out droneInfo.droneStoredGas))
                            {
                                droneInfo.droneStoredGas = 0.0;
                            }
                            if (!double.TryParse(recivedDroneOre, out droneInfo.droneStoredOre))
                            {
                                droneInfo.droneStoredOre = 0.0;
                            }
                            if (!bool.TryParse(recivedDroneCargoFull, out droneInfo.droneCargoFull))
                            {
                                droneInfo.droneCargoFull = false;
                            }
                            if (!bool.TryParse(recievedDroneRechargeRequest, out droneInfo.droneRechargeRequest))
                            {
                                droneInfo.droneRechargeRequest = false;
                            }
                            if (!bool.TryParse(recievedDroneAutoDockEnabled, out droneInfo.droneAutodockEnabled))
                            {
                                droneInfo.droneAutodockEnabled = false;
                            }
                            if (!bool.TryParse(recievedDroneDockingReady, out droneInfo.droneDockingReady))
                            {
                                droneInfo.droneDockingReady = false;
                            }
                            droneInfo.droneGPSCoordinatesDS = remote_control_actual.GetPosition();
                            droneInfo.droneReady = false;
                            droneInfo.droneMining = false;
                            droneInfo.droneMustWait = true;
                            droneInfo.droneControlSequence = 0;
                            droneInfo.droneRecallSequence = 0;
                            droneInfo.droneRecallList = false;
                            droneInfo.droneResetFunc = false;
                            droneInfo.droneTransmissionOutput = "";
                            droneInfo.droneTransmissionStatus = true;
                            droneInfo.droneAssignsCount = 0;
                            droneInfo.droneAssignedCoordinates = false;
                            dcs.Add(0.0);
                            dst.Add(true);

                            droneDataList.Add(droneInfo);
                            Confirmed_Drone_Message = false;
                            recievedDroneNameIndex = droneDataList.Count -1;
                        }
                    }
                }
            }
            #endregion
        }

        void update_display()  // Extracted from drone_processing
        {
            int startInstructions = Runtime.CurrentInstructionCount;
            dp_txm.Clear().EnsureCapacity(512); // ~400-600 chars typical
            dp_txm.AppendLine($"GMDC {ver} Running {icon}")
                  .AppendLine($"------------------------------")
                  .AppendLine($" ")
                   .AppendLine($"Total drones detected: {droneDataList.Count}")
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
                  .AppendLine(m_gps_crds.ToString());
            if (align_target_valid) dp_txm.AppendLine("Secondary/Asteroid:").AppendLine(align_gps_coords.ToString());

            if (display_tag_main.Count > 0 && sM != null) sM.WriteText(dp_txm);
            Echo($"update_display: {Runtime.CurrentInstructionCount - startInstructions}");
        }




        //program end
    }
}

