using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
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
        // R e a d m e
        // -----------
        // GMDC Drone controller
        // 

        #region mdk preserve
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }
        string drone_tag = "SWRM_D";
        double drone_length = 2.6;
        //display surface indexes
        int srfM = 0;
        int srfL = 0;
        int srfD = 0;
        int drones_per_screen = 8;
        int undock_delay_time = 30;
        #endregion
        //statics
        string ver = "V0.321A";
        string comms = "Comms";
        string MainS = "Main";
        string DroneS = "Drone";
        string IntfS = "Interface";
        string LstS = "List";
        string dspy = "Display";
        int clbs = 44;
        double bclu = 30.0;
        string tx_chan;
        bool launched_drone_status = false;
        int ft_ftr = 1;
        int hd_lm = 10;
        int skp_br = 0;
        double grdsz;
        int nPtsY;
        int nPtsX;
        int t_lim = 10;
        int p_lim = 10;
        bool c_gd = false;
        int mx_act_dn;
        string dat_in;
        string dat_in2;
        double bclm = 1.0;
        int gnm_prsp = 0;
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
        string intfc_tag = "";
        string c_dist;
        double drl_len;
        bool dat_vld = false;
        bool can_run = false;
        bool can_reset = false;
        bool can_tx = false;
        bool must_recall = false;
        bool mstfrz = false;
        bool can_init = false;
        bool found = false;
        bool general_reset;
        bool mining_grid_valid = false;
        List<bool> drone_must_wait;
        double ign_dpth = 0.0;
        double safe_dstvl = 0.0;
        bool tgt_vld = false;
        bool astd_vld = false;
        string command_ask;
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
        int t_mne_runs = 1;
        int total_drones_mining = 0;
        int t_drn_dckg = 0;
        int t_drn_dck = 0;
        int bores_remaining;
        bool flto = false;
        int fltc = 0;
        IMyRadioAntenna ant_act;
        IMyLightingBlock lss_at;
        IMyRemoteControl rm_ctl_act;
        IMyProgrammableBlock pb_i_act;
        Vector3D m_gps_crds;
        Vector3D c_gps_crds;
        Vector3D next_gps_crds;
        Vector3D t_gps_crds;
        Vector3D a_gps_crds;
        Vector3D planeNrml;
        StringBuilder mcd_nw;
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
        List<int> drone_assigns_count;
        List<double> dcs;
        List<bool> drone_assigned_coordinates;
        List<bool> drone_recall_list;
        List<Vector3D> drone_gps_coordinates_ds;
        List<int> drone_control_sequence;
        List<int> drone_recall_sequence;
        List<bool> drone_reset_func;
        List<string> dt_out;
        List<Vector3D> grid_bore_positions;
        List<Vector3D> dspl;
        List<bool> grid_bore_occupied;
        List<bool> grid_bore_finished;
        List<string> cl;
        List<string> cl2;
        List<int> tla;
        List<int> rst;
        List<string> fct;
        List<bool> dst;
        List<bool> txmt;
        int cbval = 0;
        bool clbt = false;
        int bores_completed;
        int gps_grid_position_value = -1;
        string drone_namer = "";
        StringBuilder drninf;
        StringBuilder dp_txm;
        StringBuilder dp_txl;
        StringBuilder jxt;
        List<bool> drone_mining;
        bool stp_cmpl = false;
        bool time_delay = false;
        int time_count = 0;
        int pngt_count = 0;
        bool rdy_flg = false;
        int rst_count = 0;
        int stts_cnt = 0;
        string stts = "Idle";
        string replyC = "reply";
        string prospC = "prospector";
        string command_recall = "recall";
        string command_operate = "operate";
        bool must_eject = false;
        bool run_arg = false;
        bool rnw_hdr = true;
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
        decimal dps_r_d = 0.0m;
        List<IMyRemoteControl> rm_ctl_all;
        List<IMyRemoteControl> rm_ctl_tag;
        List<IMyRadioAntenna> at_all;
        List<IMyRadioAntenna> at_tg;
        List<IMyLightingBlock> lts_all;
        List<IMyLightingBlock> lts_sys_tg;
        List<IMyTerminalBlock> ds_all;
        List<IMyTerminalBlock> ds_tg_main;
        List<IMyTerminalBlock> ds_tg_lst;
        List<IMyTerminalBlock> ds_tg_drn;
        List<IMyProgrammableBlock> pb_all;
        List<IMyProgrammableBlock> pb_tg;
        List<MyIGCMessage> myIGCMsgs;
        IMyTextSurface sD;
        IMyTextSurface sM;
        IMyTextSurface sL;
        StringBuilder sb;
        int t_dn_dmg = 0;
        int t_dn_unk = 0;
        int t_dn_ok = 0;
        string g1;
        string g2;
        int di = 0;
        int total_drones_undocking = 0;
        int undock_timer = 0;
        bool drones_undocking = false;
        bool can_loading = false;

        public void Save()
        {
            sb = new StringBuilder();
            if (grid_bore_finished.Count > 0 && grid_bore_occupied.Count > 0)
            {
                for (int i = 0; i < grid_bore_finished.Count; i++)
                {
                    if (grid_bore_finished[i])
                    {
                        g1 = "1";
                    }
                    else
                    {
                        g1 = "0";
                    }
                    if (grid_bore_occupied[i])
                    {
                        g2 = "1";
                    }
                    else
                    {
                        g2 = "0";
                    }
                    sb.Append(g1 + ":" + g2 + ";");
                }
                Storage = sb.ToString();
                sb.Clear();
            }
        }


        public void Main(string argt, UpdateType updateSource)
        {
            IMyGridTerminalSystem gts = GridTerminalSystem as IMyGridTerminalSystem;
            if (!stp_cmpl)
            {
                ant_tg = "[" + drone_tag + " " + comms + "]";
                lt_tag = "[" + drone_tag + " " + comms + "]";
                dp_mn_tag = "[" + drone_tag + " " + MainS + " " + dspy + "]";
                dp_drn_tag = "[" + drone_tag + " " + DroneS + " " + dspy + "]";
                dp_lst_tag = "[" + drone_tag + " " + LstS + " " + dspy + "]";
                intfc_tag = "[" + drone_tag + " " + IntfS + "]";
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
                grid_bore_positions = new List<Vector3D>();
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
                dt_out = new List<string>();
                drone_recall_sequence = new List<int>();
                drone_ready = new List<bool>();
                drone_must_wait = new List<bool>();
                drone_recall_list = new List<bool>();
                drone_assigns_count = new List<int>();
                rm_ctl_all = new List<IMyRemoteControl>();
                rm_ctl_tag = new List<IMyRemoteControl>();
                myIGCMsgs = new List<MyIGCMessage>();
                cl = new List<string>();
                cl2 = new List<string>();
                tla = new List<int>();
                rst = new List<int>();
                fct = new List<string>();
                dcs = new List<double>();
                dst = new List<bool>();
                txmt = new List<bool>();
                mcd_nw = new StringBuilder();
                dp_txm = new StringBuilder();
                dp_txl = new StringBuilder();
                drninf = new StringBuilder();
                drone_reset_func = new List<bool>();
                c = new StringBuilder();
                jxt = new StringBuilder();
                for (int i = 0; i < 12; i++)
                {
                    cl.Add("");
                    cl2.Add("");
                    tla.Add(0);
                    rst.Add(0);
                    fct.Add("");
                }
                gts.GetBlocksOfType<IMyRemoteControl>(rm_ctl_all, b => b.CubeGrid == Me.CubeGrid);
                for (int i = 0; i < rm_ctl_all.Count; i++)
                {
                    if (rm_ctl_all[i].CustomName.Contains(ant_tg))
                    {
                        rm_ctl_tag.Add(rm_ctl_all[i]);
                    }
                }
                rm_ctl_all.Clear();
                at_all = new List<IMyRadioAntenna>();
                at_tg = new List<IMyRadioAntenna>();
                gts.GetBlocksOfType<IMyRadioAntenna>(at_all, b => b.CubeGrid == Me.CubeGrid);
                for (int i = 0; i < at_all.Count; i++)
                {
                    if (at_all[i].CustomName.Contains(ant_tg))
                    {
                        at_tg.Add(at_all[i]);
                    }
                }
                at_all.Clear();
                lts_all = new List<IMyLightingBlock>();
                lts_sys_tg = new List<IMyLightingBlock>();
                gts.GetBlocksOfType<IMyLightingBlock>(lts_all, b => b.CubeGrid == Me.CubeGrid);
                for (int i = 0; i < lts_all.Count; i++)
                {
                    if (lts_all[i].CustomName.Contains(lt_tag))
                    {
                        lts_sys_tg.Add(lts_all[i]);
                    }
                }
                lts_all.Clear();
                ds_all = new List<IMyTerminalBlock>();
                ds_tg_main = new List<IMyTerminalBlock>();
                ds_tg_lst = new List<IMyTerminalBlock>();
                ds_tg_drn = new List<IMyTerminalBlock>();
                gts.GetBlocksOfType<IMyTerminalBlock>(ds_all, b => b.CubeGrid == Me.CubeGrid);
                for (int i = 0; i < ds_all.Count; i++)
                {
                    if (ds_all[i].CustomName.Contains(dp_mn_tag))
                    {
                        ds_tg_main.Add(ds_all[i]);
                    }
                    if (ds_all[i].CustomName.Contains(dp_drn_tag))
                    {
                        ds_tg_drn.Add(ds_all[i]);
                    }
                    if (ds_all[i].CustomName.Contains(dp_lst_tag))
                    {
                        ds_tg_lst.Add(ds_all[i]);
                    }
                }
                ds_all.Clear();
                pb_all = new List<IMyProgrammableBlock>();
                pb_tg = new List<IMyProgrammableBlock>();
                gts.GetBlocksOfType<IMyProgrammableBlock>(pb_all, b => b.CubeGrid == Me.CubeGrid);
                for (int i = 0; i < pb_all.Count; i++)
                {
                    if (pb_all[i].CustomName.Contains(intfc_tag))
                    {
                        pb_tg.Add(pb_all[i]);
                    }
                }
                pb_all.Clear();
                stp_cmpl = true;
                Echo("Setup complete!");
            }

            if (rm_ctl_tag.Count <= 0 || rm_ctl_tag[0] == null)
            {
                Echo($"remote control with tag: '{ant_tg}' not found.");
                return;
            }
            rm_ctl_act = rm_ctl_tag[0];
            if (at_tg.Count <= 0 || at_tg[0] == null)
            {
                Echo($"Antenna with tag: '{ant_tg}' not found.");
                return;
            }
            ant_act = at_tg[0];

            if (lts_sys_tg.Count <= 0 || lts_sys_tg[0] == null)
            {
                Echo($"light with tag: '{lt_tag}' not found.");
                return;
            }
            lss_at = lts_sys_tg[0];
            lss_at.SetValue("Color", Cred);
            if (ds_tg_main.Count <= 0 || ds_tg_main[0] == null)
            {
                Echo($"Display with tag '{dp_mn_tag}' not found");
            }
            if (ds_tg_main.Count > 0 && ds_tg_main[0] != null)
            {
                sM = ((IMyTextSurfaceProvider)ds_tg_main[0]).GetSurface(srfM);
                if (sM.ContentType != ContentType.TEXT_AND_IMAGE)
                {
                    sM.ContentType = ContentType.TEXT_AND_IMAGE;
                }
            }
            if (sM == null)
            {
                Echo($"Panel:'{srfM}' on '{dp_mn_tag}' not found");
            }
            if (ds_tg_lst.Count <= 0 || ds_tg_lst[0] == null)
            {
                Echo($"Display with tag '{dp_lst_tag}' not found");
            }
            if (ds_tg_lst.Count > 0 && ds_tg_lst[0] != null)
            {
                sL = ((IMyTextSurfaceProvider)ds_tg_lst[0]).GetSurface(srfL);
                if (sL.ContentType != ContentType.TEXT_AND_IMAGE)
                {
                    sL.ContentType = ContentType.TEXT_AND_IMAGE;
                }
            }
            if (sL == null)
            {
                Echo($"Panel:'{srfL}' on '{dp_lst_tag}' not found");
            }
            if (ds_tg_drn.Count <= 0 || ds_tg_drn[0] == null)
            {
                Echo($"Display with tag '{dp_drn_tag}' not found");
            }

            if (ds_tg_drn.Count > 0 && ds_tg_drn[0] != null)
            {
                sD = ((IMyTextSurfaceProvider)ds_tg_drn[0]).GetSurface(srfD);
                if (sD.ContentType != ContentType.TEXT_AND_IMAGE)
                {
                    sD.ContentType = ContentType.TEXT_AND_IMAGE;
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
            if (pb_tg.Count > 0)
            {
                pb_i_act = pb_tg[0];
                can_intf = true;
                it_ag = pb_i_act.CustomData.ToString();
                Echo($"Interface PB: {intfc_tag}");
                Echo($"Display command: {it_ag}");
            }
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
                else
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
            IMyBroadcastListener listen = IGC.RegisterBroadcastListener(rx_ch);
            IMyBroadcastListener listen_prspt = IGC.RegisterBroadcastListener(rx_ch_2);
            myIGCMsgs.Clear();

            Echo($"GMDC {ver} Running..");
            if (argt == "setup" && stp_cmpl)
            {
                stp_cmpl = false;
                argt = "";
                Echo("Running Setup..");
            }
            if (argt.Contains("run") || i_run)
            {
                can_run = true;
                can_reset = false;
                can_tx = true;
                must_recall = false;
                can_init = false;
                mstfrz = false;
                command_ask = "Run";
            }
            if (argt.Contains("reset") || i_res)
            {
                can_reset = true;
                must_eject = false;
                can_run = false;
                can_tx = true;
                must_recall = false;
                can_init = false;
                mstfrz = false;
                command_ask = "Reset";
                current_gps_idx = 0;
            }
            if (argt.Contains("stop") || i_stop)
            {
                can_tx = false;
                must_eject = false;
                can_run = false;
                can_reset = false;
                must_recall = false;
                can_init = false;
                mstfrz = false;
                command_ask = "Stop";
            }
            if (argt.Contains("recall") || i_recall)
            {
                must_recall = true;
                must_eject = false;
                can_reset = false;
                can_tx = true;
                can_run = false;
                mstfrz = false;
                command_ask = "Recall";
                current_gps_idx = 0;
            }
            if (argt.Contains("init") || i_init)
            {
                must_recall = false;
                must_eject = false;
                can_reset = false;
                can_tx = false;
                can_run = false;
                can_init = true;
                mstfrz = false;
                command_ask = "Init";
                current_gps_idx = 0;
                r_gps_idx = current_gps_idx;
                Storage = null;
            }
            if (argt.Contains("eject") || i_eject)
            {
                must_recall = false;
                must_eject = true;
                can_reset = false;
                can_tx = true;
                can_run = false;
                mstfrz = false;
                command_ask = "Eject";
                current_gps_idx = 0;
            }
            if (argt.Contains("freeze") || i_frz)
            {
                can_tx = true;
                mstfrz = true;
                must_eject = false;
                can_run = false;
                can_reset = false;
                must_recall = false;
                can_init = false;
                command_ask = "Freeze";
            }
            if (must_eject || must_recall || mstfrz)
            {
                run_arg = true;
            }
            else
            {
                run_arg = false;
            }
            if (can_init && c_gd)
            {
                c_gd = false;
                mining_grid_valid = false;
                current_gps_idx = 0;
                gps_grid_position_value = -1;
            }
            if (can_loading)
            {
                can_loading = false;
            }

            if (Me.CustomData != null && Me.CustomData != "")
            {
                dat_vld = true;
            }
            else dat_vld = false;
            if (dat_vld)
            {
                GtCstData();

            }
            if (ant_act != null)
            {
                if (drone_name.Count == 0 && !pinged || drone_name.Count > 0 && !pinged)
                {
                    IGC.SendBroadcastMessage(tx_ping_channel, p_cht, TransmissionDistance.TransmissionDistanceMax);
                    pinged = true;
                    pngt_count = 0;
                }
                if (listen.HasPendingMessage)
                {
                    MyIGCMessage new_msg = listen.AcceptMessage();
                    dat_in = new_msg.Data.ToString();
                    GtMsgData(dat_in);

                }
                if (listen_prspt.HasPendingMessage)
                {
                    MyIGCMessage new_msg_2 = listen_prspt.AcceptMessage();
                    dat_in2 = new_msg_2.Data.ToString();
                    rm_ctl_act.CustomData = dat_in2;
                    gnm_prsp = 1;
                    c_gd = false;
                }
            }
            if (recieved_drone_id_name != null && recieved_drone_id_name != "")
            {
                found = false;
                if (drone_name.Count <= 0)
                {
                    drone_name.Add(recieved_drone_id_name);
                    drone_damage_state.Add(rc_dds);
                    drone_tunnel_complete.Add(rc_tnl_end);
                    drone_control_status.Add(rc_dn_sts);
                    drone_dock_status.Add(recieved_drone_dock);
                    drone_undock_status.Add(recieved_drone_undock);
                    drone_autopilot_status.Add(recived_drone_autopilot);
                    drone_gps_grid_list_position.Add(-1);
                    drone_gps_coordinates_ds.Add(rm_ctl_act.GetPosition());
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
                    drone_control_sequence.Add(0);
                    drone_recall_sequence.Add(0);
                    dt_out.Add("");
                    drone_ready.Add(false);
                    drone_must_wait.Add(true);
                    dcs.Add(0.0);
                    dst.Add(true);
                    txmt.Add(true);
                    drone_recall_list.Add(false);
                    drone_reset_func.Add(false);
                    drone_assigns_count.Add(0);
                }
                if (drone_name.Count > 0)
                {

                    for (int i = 0; i < drone_name.Count; i++)
                    {
                        found = false;
                        string nametag = drone_name[i];
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
                            txmt[i] = true;
                            if (dcs[i] <= bclm)
                            {
                                dst[i] = false;
                            }
                            if (dcs[i] > bclm)
                            {
                                dst[i] = true;
                            }
                            break;
                        }
                        if (i == (drone_name.Count - 1) && !nametag.Equals(recieved_drone_id_name) && !found)
                        {
                            drone_name.Add(recieved_drone_id_name);
                            drone_damage_state.Add(rc_dds);
                            drone_tunnel_complete.Add(rc_tnl_end);
                            drone_control_status.Add(rc_dn_sts);
                            drone_dock_status.Add(recieved_drone_dock);
                            drone_undock_status.Add(recieved_drone_undock);
                            drone_autopilot_status.Add(recived_drone_autopilot);
                            drone_gps_coordinates_ds.Add(rm_ctl_act.GetPosition());
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
                            dt_out.Add("");
                            drone_ready.Add(false);
                            drone_must_wait.Add(true);
                            dcs.Add(0.0);
                            dst.Add(true);
                            txmt.Add(true);
                            drone_recall_list.Add(false);
                            drone_reset_func.Add(false);
                            drone_assigns_count.Add(0);
                        }
                    }
                }
            }

            GtRCData();
            if (gnm_prsp == 1)
            {
                GtRCData();
                if (tgt_vld)
                {
                    mcd_nw.Clear();
                    mcd_nw.Append("GPS" + ":" + "PDT" + ":" + Math.Round(t_gps_crds.X, 2) + ":" + Math.Round(t_gps_crds.Y, 2) + ":" + Math.Round(t_gps_crds.Z, 2) + ":" + "#FF75C9F1" + ":" + "5.0" + ":" + "10.0" + ":" + "1" + ":" + "1" + ":" + "0" + ":" + "False" + ":" + "1" + ":" + "10" + ":" + "0" + ":");
                }
                Me.CustomData = mcd_nw.ToString();
                gnm_prsp = 0;
                c_gd = false;
            }
            if (drone_name.Count > 0)
            {
                mx_act_dn = drone_name.Count - ft_ftr;
                if (mx_act_dn <= 1)
                {
                    mx_act_dn = 1;
                }
                if (mx_act_dn > hd_lm)
                {
                    mx_act_dn = hd_lm;
                }
            }
            GtCstData();
            if (nPtsY == 0 && !c_gd || nPtsX == 0 && !c_gd || grdsz == 0 && !c_gd)
            {
                grid_bore_positions.Clear();
                grid_bore_occupied.Clear();
                grid_bore_finished.Clear();
                grid_bore_occupied = new List<bool>();
                grid_bore_finished = new List<bool>();
                c_gd = true;
                c_gps_crds = m_gps_crds;
                grid_bore_occupied.Add(false);
                grid_bore_finished.Add(false);
                grid_bore_positions.Add(c_gps_crds);
                t_mne_runs = grid_bore_positions.Count;
                current_gps_idx = 0;
                if (rdy_flg)
                {
                    rdy_flg = false;
                }
            }
            if (!c_gd)
            {
                if (rdy_flg)
                {
                    rdy_flg = false;
                }
                grid_bore_positions.Clear();
                Vector3D gravity = rm_ctl_act.GetNaturalGravity();
                if (astd_vld)
                {
                    planeNrml = ((m_gps_crds - a_gps_crds));
                }
                if (!astd_vld)
                {
                    planeNrml = gravity;
                }
                Vector3D centerPoint = m_gps_crds;
                planeNrml.Normalize();
                Vector3D perpendicularVector = Vector3D.CalculatePerpendicularVector(planeNrml);
                perpendicularVector.Normalize();
                grid_bore_positions = GenGrdPosits(centerPoint, planeNrml, grdsz, nPtsX, nPtsY);
                if (Storage != null && Storage != "" && !c_gd)
                {
                    grid_bore_finished.Clear();
                    grid_bore_occupied.Clear();
                    //added from init
                    current_gps_idx = 0;
                    r_gps_idx = current_gps_idx;
                    GtStrD();
                    Echo("Grid positions restored");
                    can_loading = true;
                    Storage = null;
                }
                c_gd = true;
                t_mne_runs = (nPtsX * nPtsY);
                if (nPtsY == 0 || nPtsY == 0 || grdsz == 0 || nPtsY == 0 && nPtsY == 0 && grdsz == 0)
                {
                    mining_grid_valid = false;
                }
                if (nPtsY > 0 && nPtsY > 0 && grdsz > 0)
                {
                    mining_grid_valid = true;
                }
                if (!mining_grid_valid)
                {
                    t_mne_runs = 1;
                }
                t_mne_sq_cmp = 0;
                bores_completed = 0;
                current_gps_idx = 0;
            }
            if (drone_name.Count > 0 && c_gd)
            {

                if (time_delay)
                {
                    time_delay = false;
                    time_count = 0;
                    if (must_recall)
                    {
                        IGC.SendBroadcastMessage(tx_recall_channel, command_recall, TransmissionDistance.TransmissionDistanceMax);
                    }
                    else
                    {
                        IGC.SendBroadcastMessage(tx_recall_channel, command_operate, TransmissionDistance.TransmissionDistanceMax);
                    }
                    if (must_recall && current_gps_idx > 0)
                    {
                        current_gps_idx = 0;
                    }
                    for (int i = 0; i < drone_name.Count; i++)
                    {
                        if (can_init || can_reset || drone_reset_func[i] || can_loading)
                        {
                            general_reset = true;
                        }
                        else general_reset = false;
                        di = i;
                        fltc = CntTrueVls(dst);
                        if (fltc < drone_name.Count)
                        {
                            flto = true;
                        }
                        else flto = false;

                        //recall sequence reset - global
                        if (!drone_recall_list[i] || can_reset || can_init || can_loading)
                        {
                            drone_recall_sequence[i] = 0;
                        }
                        dp_txm.Clear();
                        bores_completed = CntTrueVls(grid_bore_finished);

                        if (mining_grid_valid)
                        {
                            bores_remaining = t_mne_runs - bores_completed;
                        }
                        if (!mining_grid_valid)
                        {
                            bores_remaining = t_mne_runs - bores_completed;
                        }
                        if (drone_gps_grid_list_position[i] > -1 && !drone_assigned_coordinates[i])
                        {
                            drone_gps_grid_list_position[i] = -1;
                        }
                        //if undocked request local recall sequence flag to ON
                        if (drone_gps_grid_list_position[i] == -1 && !drone_assigned_coordinates[i] && drone_undock_status[i] == "True" && drone_dock_status[i] == "False" && !drone_recall_list[i] && !must_eject || drone_gps_grid_list_position[i] == -1 && !drone_assigned_coordinates[i] && drone_undock_status[i] == "False" && drone_dock_status[i] == "False" && !drone_recall_list[i] && !must_eject)
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
                        total_drones_mining = CntTrueVls(drone_mining);
                        t_drn_dckg = CntStsVls(drone_control_status, "Docking");
                        t_drn_dck = CntStsVls(drone_control_status, "Docked");
                        t_dn_dmg = CntStsVls(drone_damage_state, "DMG");
                        t_dn_unk = CntStsVls(drone_damage_state, "UNK");
                        t_dn_ok = CntStsVls(drone_damage_state, "OK");
                        if (grid_bore_occupied.Count > 0)
                        {
                            //why 1 less than
                            for (int l = 0; l < grid_bore_occupied.Count; l++)
                            {
                                //find number of grid positions queued
                                int queued_count = CntIntVls(drone_gps_grid_list_position, l);
                                if (grid_bore_occupied[l] && queued_count == 0)
                                {
                                    grid_bore_occupied[l] = false;
                                }                                
                            }
                        }
                        if (drone_control_status[i].Contains("Docked") && drone_gps_grid_list_position[i] == -1 && drone_mining[i] && drone_control_sequence[i] == 0)
                        {
                            drone_mining[i] = false;
                        }
                        //check if drones are launching here
                        total_drones_undocking = CntIntVls(drone_control_sequence, 2);
                        if (total_drones_undocking > 0)
                        {
                            drones_undocking = true;
                        }
                        if(!drones_undocking && undock_timer != 0)
                        {
                            undock_timer = 0;
                        }

                        if (total_drones_mining >= bores_remaining && !drone_mining[i] && bores_completed <= t_mne_runs || bores_remaining == 0 && drone_mining[i] == false)
                        {
                            if (!launched_drone_status || drones_undocking)
                            {
                                drone_must_wait[i] = true;
                            }
                            if (launched_drone_status && total_drones_mining > mx_act_dn || drones_undocking)
                            {
                                drone_must_wait[i] = true;
                            }
                            if (launched_drone_status && total_drones_mining <= mx_act_dn)
                            {
                                drone_must_wait[i] = false;
                            }
                        }
                        else if (total_drones_mining < bores_remaining && bores_completed < t_mne_runs || drone_mining[i] && total_drones_mining <= bores_remaining)
                        {
                            if (!launched_drone_status)
                            {
                                drone_must_wait[i] = false;
                            }
                            if (launched_drone_status && total_drones_mining < mx_act_dn)
                            {
                                drone_must_wait[i] = false;
                            }

                            if (launched_drone_status && total_drones_mining > mx_act_dn || drones_undocking)
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
                            if (launched_drone_status && total_drones_mining >= mx_act_dn || drones_undocking)
                            {
                                drone_must_wait[i] = true;
                            }
                            if (launched_drone_status && total_drones_mining < mx_act_dn || drones_undocking)
                            {
                                drone_must_wait[i] = true;
                            }
                        }
                        if (drone_gps_grid_list_position[i] > -1 && drone_gps_grid_list_position[i] < grid_bore_positions.Count)
                        {
                            if (grid_bore_occupied[drone_gps_grid_list_position[i]] && !drone_mining[i])
                            {
                                if (!launched_drone_status || drones_undocking)
                                {
                                    drone_must_wait[i] = true;
                                }
                                if (launched_drone_status && total_drones_mining >= mx_act_dn || drones_undocking)
                                {
                                    drone_must_wait[i] = true;
                                }
                                if (launched_drone_status && total_drones_mining < mx_act_dn || drones_undocking)
                                {
                                    drone_must_wait[i] = true;
                                }
                            }
                            else if (bores_completed < t_mne_runs && !grid_bore_occupied[drone_gps_grid_list_position[i]] && !grid_bore_finished[drone_gps_grid_list_position[i]] && !drone_mining[i])
                            {
                                if (!launched_drone_status)
                                {
                                    drone_must_wait[i] = false;
                                }
                                if (launched_drone_status && total_drones_mining < mx_act_dn)
                                {
                                    drone_must_wait[i] = false;
                                }
                                if (launched_drone_status && total_drones_mining >= mx_act_dn || drones_undocking)
                                {
                                    drone_must_wait[i] = true;
                                }
                            }
                            if (!grid_bore_finished[drone_gps_grid_list_position[i]])
                            {
                                int queued_count = CntIntVls(drone_gps_grid_list_position, drone_gps_grid_list_position[i]);
                                if (grid_bore_occupied[drone_gps_grid_list_position[i]] && queued_count == 0)
                                {
                                    grid_bore_occupied[drone_gps_grid_list_position[i]] = false;
                                }

                            }
                        }
                        dp_txm.Append("Drone Controller Status" + " - GMDC " + ver);
                        dp_txm.Append('\n');

                        if (drone_control_sequence[i] == 12)
                        {
                            gps_grid_position_value = -1;
                            drone_mining[i] = false;
                            dp_txm.Append('\n');
                            dp_txm.Append("Mining seq. complete"); ;
                        }
                        if (grid_bore_positions.Count > 0)
                        {
                            dp_txm.Append('\n');
                            dp_txm.Append("Grid pos: " + grid_bore_positions.Count);
                            dp_txm.Append('\n');
                            dp_txm.Append("Grid dist: " + cst_dt7 + "m #X: " + cst_dt8 + " #Y: " + cst_dt9);
                            dp_txm.Append('\n');
                            dp_txm.Append("Grid OK: " + mining_grid_valid);
                            dp_txm.Append('\n');
                            dp_txm.Append("Bores: " + grid_bore_positions.Count + " Remain: " + bores_remaining + "  Skip: " + skp_br);
                            dp_txm.Append('\n');
                        }
                        else
                        {
                            dp_txm.Append('\n');
                            dp_txm.Append("Bores: " + t_mne_runs + " Remaining: " + bores_remaining);
                        }
                        if (t_mne_runs > 0)
                        {
                            dp_txm.Append('\n');
                            dp_txm.Append("Current mine idx: " + r_gps_idx + " of " + (t_mne_runs - 1) + " (" + bores_completed + ") ");
                        }
                        else
                        {
                            dp_txm.Append('\n');
                            dp_txm.Append("Current mine idx: " + current_gps_idx + " of " + (t_mne_runs - 1) + " (" + bores_completed + ") ");
                        }
                        if (current_gps_idx > t_mne_runs || !mining_grid_valid && bores_completed >= t_mne_runs || bores_remaining == 0)
                        {
                            can_run = false;
                            dp_txm.Append('\n');
                            dp_txm.Append("Mine seq. complete");
                            dp_txm.Append('\n');
                            drone_control_sequence[i] = 12;
                            current_gps_idx = 0;
                        }
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
                            if (bores_completed < t_mne_runs && mining_grid_valid != false && !drone_assigned_coordinates[i] && drone_must_wait[i] == false)
                            {

                                if (grid_bore_finished.Count > 0)
                                {
                                    if (skp_br > grid_bore_finished.Count)
                                    {
                                        skp_br = 0;
                                    }
                                    if (skp_br > 0)
                                    {
                                        for (int j = 0; j < skp_br; j++)
                                        {
                                            if (j > grid_bore_finished.Count - 1 || j > skp_br - 1)
                                            {
                                                break;
                                            }
                                            grid_bore_finished[j] = true;
                                        }
                                    }
                                    for (int k = 0; k < grid_bore_finished.Count; k++)
                                    {

                                        if (k > grid_bore_finished.Count - 1)
                                        {
                                            k = grid_bore_finished.Count - 1;
                                        }
                                        if (!grid_bore_finished[k] && !grid_bore_occupied[k])
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
                                    drone_gps_coordinates_ds[i] = grid_bore_positions[gps_grid_position_value];
                                    drone_gps_grid_list_position[i] = gps_grid_position_value;
                                }
                                else
                                {
                                    gps_grid_position_value = drone_gps_grid_list_position[i];
                                    drone_gps_coordinates_ds[i] = grid_bore_positions[gps_grid_position_value];
                                }
                                if (!mining_grid_valid)
                                {
                                    t_mne_runs = 1;
                                    drone_gps_coordinates_ds[i] = m_gps_crds;
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
                                t_mne_runs = 1;
                                drone_gps_coordinates_ds[i] = m_gps_crds;
                                drone_assigned_coordinates[i] = true;
                                gps_grid_position_value = 0;
                                current_gps_idx = 0;
                            }
                            if (drone_gps_grid_list_position[i] > -1)
                            {
                                if (grid_bore_occupied[drone_gps_grid_list_position[i]] && !drone_mining[i])
                                {
                                    drone_must_wait[i] = true;
                                }
                                else if (total_drones_mining < bores_remaining && bores_completed < t_mne_runs || !grid_bore_occupied[drone_gps_grid_list_position[i]] && !grid_bore_finished[drone_gps_grid_list_position[i]] && !drone_mining[i])
                                {
                                    drone_must_wait[i] = false;
                                }
                                if (bores_completed != t_mne_runs && !drone_must_wait[i])
                                {
                                    drone_control_sequence[i] = 1;
                                    drone_mining[i] = true;
                                    grid_bore_occupied[drone_gps_grid_list_position[i]] = true;
                                }
                                else
                                {
                                    drone_control_sequence[i] = 0;
                                    drone_mining[i] = false;
                                }
                                if (grid_bore_finished[drone_gps_grid_list_position[i]])
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
                        cd1 = gps_grid_position_value.ToString();
                        cm = "0";
                        xp = Math.Round(drone_gps_coordinates_ds[i].X, 2).ToString();
                        yp = Math.Round(drone_gps_coordinates_ds[i].Y, 2).ToString();
                        zp = Math.Round(drone_gps_coordinates_ds[i].Z, 2).ToString();
                        cd5 = cst_dt5;
                        cd6 = (drl_len + safe_dstvl).ToString();
                        igd = (ign_dpth + safe_dstvl + drone_length).ToString();
                        if (astd_vld)
                        {
                            xp2 = Math.Round(((drone_gps_coordinates_ds[i].X - m_gps_crds.X) + a_gps_crds.X), 2).ToString();
                            yp2 = Math.Round(((drone_gps_coordinates_ds[i].Y - m_gps_crds.Y) + a_gps_crds.Y), 2).ToString();
                            zp2 = Math.Round(((drone_gps_coordinates_ds[i].Z - m_gps_crds.Z) + a_gps_crds.Z), 2).ToString();
                        }
                        else
                        {
                            xp2 = "";
                            yp2 = "";
                            zp2 = "";
                        }
                        if (drone_control_sequence[i] == 1 && drone_assigned_coordinates[i] && !drone_must_wait[i] && !run_arg || drone_control_sequence[i] == 2 && drone_control_status[i] == "Docked Idle" && drone_dock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 2;
                            drone_mining[i] = true;
                            grid_bore_occupied[drone_gps_grid_list_position[i]] = true;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "7";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 2 && drone_control_status[i] == "Undocked" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg || drone_control_sequence[i] == 2 && drone_control_status[i] == "Docking" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 3;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }

                        if (drone_control_sequence[i] == 2 && drone_control_status[i] == "Undocking" && drone_dock_status[i] == "False" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg && dcs[i] <= bclu)
                        {
                            drone_control_sequence[i] = 13;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 13 && drone_control_status[i] == "Idle" && drone_dock_status[i] == "False" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg || drone_control_sequence[i] == 5 && drone_control_status[i] == "Docking" && drone_dock_status[i] == "False" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg && dcs[i] <= bclu)
                        {
                            drone_control_sequence[i] = 8;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "6";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 3 && drone_control_status[i] == "Idle" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 4;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "4";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 4 && drone_control_status[i] == "Nav End" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 5;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 4 && drone_control_status[i] == "Docked Idle" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 1;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }

                        if (drone_control_sequence[i] == 5 && drone_control_status[i] == "Idle" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 6;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "2";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }

                        if (drone_control_sequence[i] == 6 && drone_control_status[i] == "Nav End" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 7;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 7 && drone_control_status[i] == "Idle" && drone_undock_status[i] == "True" && drone_assigned_coordinates[i] && drone_mining[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 8;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "5";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] >= 8 && drone_control_status[i].Contains("Docked") && drone_mining[i] || drone_control_sequence[i] == 4 && drone_control_status[i].Contains("Docked") && drone_mining[i])
                        {
                            grid_bore_occupied[drone_gps_grid_list_position[i]] = false;
                        }
                        if (drone_control_sequence[i] >= 8 && drone_control_status[i].Contains("Dock") && drone_mining[i] && drone_tunnel_complete[i] == "True")
                        {
                            grid_bore_finished[drone_gps_grid_list_position[i]] = true;
                        }
                        if (drone_control_sequence[i] == 8 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 1;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 8 && !drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 8 && !drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] >= 1 && drone_control_sequence[i] <= 4 && !drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 0;
                            drone_assigned_coordinates[i] = false;
                            drone_mining[i] = false;
                            gps_grid_position_value = -1;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 8 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 9;
                            grid_bore_finished[drone_gps_grid_list_position[i]] = true;

                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 9 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && can_run && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 9 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && drone_assigned_coordinates[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 10;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 10 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && general_reset && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 10 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg || drone_control_sequence[i] == 0 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "True" && drone_assigned_coordinates[i] && !run_arg)
                        {
                            drone_control_sequence[i] = 11;
                            drone_tunnel_complete[i] = "False";
                            grid_bore_finished[drone_gps_grid_list_position[i]] = true;
                            t_mne_sq_cmp++;
                            gps_grid_position_value = -1;
                            drone_reset_func[i] = false;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "8";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 11 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && t_mne_sq_cmp <= t_mne_runs && mining_grid_valid && !run_arg)
                        {
                            drone_control_sequence[i] = 0;
                            drone_assigned_coordinates[i] = false;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (drone_control_sequence[i] == 11 && drone_control_status[i].Contains("Docked") && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && current_gps_idx < t_mne_runs && drone_assigned_coordinates[i] && t_mne_sq_cmp > t_mne_runs && !run_arg || drone_control_sequence[i] == 11 && drone_ready[i] && drone_dock_status[i] == "True" && drone_tunnel_complete[i] == "False" && drone_assigned_coordinates[i] && mining_grid_valid == false && t_mne_sq_cmp >= t_mne_runs && !run_arg)
                        {
                            drone_control_sequence[i] = 12;
                            drone_assigned_coordinates[i] = false;
                            gps_grid_position_value = -1;
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
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
                            cd1 = gps_grid_position_value.ToString();
                            cm = "8";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
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
                            cd1 = gps_grid_position_value.ToString();
                            cm = "0";
                            cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                            dt_out[i] = c.ToString();
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (must_recall && !drone_recall_list[i] && !must_eject)
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
                                cd1 = gps_grid_position_value.ToString();
                                cm = "0";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();
                            }
                            if (drone_recall_sequence[i] == 2 && drone_control_status[i] == "Idle")
                            {
                                drone_recall_sequence[i] = 3;
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "1";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();

                            }
                            if (drone_recall_sequence[i] == 3 && drone_control_status[i] == "Nav End")
                            {
                                drone_recall_sequence[i] = 4;
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "0";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();

                            }
                            if (drone_recall_sequence[i] == 3 && drone_control_status[i] == "Nav" && drone_gps_grid_list_position[i] == -1 || drone_recall_sequence[i] == 3 && drone_control_status[i] == "Idle" && drone_gps_grid_list_position[i] >= -1)
                            {
                                drone_recall_sequence[i] = 4;
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "0";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();

                            }

                            if (drone_recall_sequence[i] == 4 && drone_control_status[i] == "Idle")
                            {
                                drone_recall_sequence[i] = 5;
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "6";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();

                            }
                            if (drone_recall_sequence[i] == 4 && drone_control_status[i] == "Idle")
                            {
                                drone_recall_sequence[i] = 5;
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "6";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();

                            }
                            if (drone_recall_sequence[i] == 5 && drone_control_status[i].Contains("Docked") || drone_recall_sequence[i] == 0 && drone_control_status[i].Contains("Docked"))
                            {
                                drone_recall_sequence[i] = 0;
                                drone_assigned_coordinates[i] = false;
                                drone_recall_list[i] = false;
                                drone_mining[i] = false;
                                gps_grid_position_value = -1;
                                drone_reset_func[i] = true;
                                cd1 = gps_grid_position_value.ToString();
                                cm = "0";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();

                            }
                            if (can_tx && txmt[i])
                            {
                                dtxt();
                                txmt[i] = false;
                            }
                        }
                        if (must_eject)
                        {
                            if (drone_control_status[i] == "Docked Idle")
                            {
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "7";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();
                                if (can_tx && txmt[i])
                                {
                                    dtxt();
                                    txmt[i] = false;
                                }
                            }
                        }
                        if (mstfrz)
                        {
                            if (drone_control_status[i] == "Undocked" || drone_control_status[i] == "Idle")
                            {
                                gps_grid_position_value = drone_gps_grid_list_position[i];
                                cd1 = gps_grid_position_value.ToString();
                                cm = "0";
                                cbld(cd1, xp, yp, zp, cd5, cm, cd6, igd, xp2, yp2, zp2);
                                dt_out[i] = c.ToString();
                                if (can_tx && txmt[i])
                                {
                                    dtxt();
                                    txmt[i] = false;
                                }
                            }
                        }
                        dp_txm.Append('\n');
                        dp_txm.Append("Total drones detected: " + drone_name.Count);
                        dp_txm.Append('\n');
                        if (!launched_drone_status)
                        {
                            dp_txm.Append("Drones active: " + total_drones_mining + "  Docking: " + t_drn_dckg + "  Docked: " + t_drn_dck);
                        }
                        if (launched_drone_status)
                        {
                            dp_txm.Append("Drones active: " + total_drones_mining + "  (Max: " + mx_act_dn + " (" + ft_ftr + ")" + ")" + "  Hard limit: " + hd_lm);
                            dp_txm.Append('\n');
                            dp_txm.Append("Docking: " + t_drn_dckg + "  Docked: " + t_drn_dck);
                        }
                        dp_txm.Append('\n');
                        dp_txm.Append('\n');
                        dp_txm.Append("Surface distance: " + safe_dstvl + "m");
                        dp_txm.Append('\n');
                        dp_txm.Append("Drill depth: " + drl_len + "m" + " (" + (drl_len + safe_dstvl) + "m)");
                        dp_txm.Append('\n');
                        dp_txm.Append("Req. ignore depth: " + ign_dpth + "m" + "  (Drone length: " + drone_length + "m)");
                        dp_txm.Append('\n');
                        dp_txm.Append("Ignore depth: " + (safe_dstvl + drone_length + ign_dpth) + "m" + "  (Drill Start: " + ((drl_len + safe_dstvl) - (ign_dpth + safe_dstvl + drone_length)) + "m)");
                        dp_txm.Append('\n');
                        dp_txm.Append('\n');
                        dp_txm.Append("Command: " + command_ask + "   Reset: " + general_reset);
                        dp_txm.Append('\n');
                        dp_txm.Append("Status: " + stts);
                        dp_txm.Append('\n');
                        dp_txm.Append('\n');
                        dp_txm.Append("Target:");
                        dp_txm.Append('\n');
                        dp_txm.Append(m_gps_crds);
                        dp_txm.Append('\n');
                        if (astd_vld)
                        {
                            dp_txm.Append("Secondary/Asteroid:");
                            dp_txm.Append('\n');
                            dp_txm.Append(a_gps_crds);
                            dp_txm.Append('\n');
                            dp_txm.Append('\n');
                        }
                        if (ds_tg_main.Count > 0 && sM != null)
                        {
                            sM.WriteText(dp_txm.ToString());
                        }
                    }
                }

            }
            if (drone_gps_grid_list_position.Count > 0 && can_reset)
            {
                rst_count = CntIntVls(drone_gps_grid_list_position, -1);
                stts_cnt = CntStsVls(drone_control_status, "Docked");
            }
            if (drone_name.Count > 0)
            {
                if (rst_count == drone_name.Count && stts_cnt == drone_name.Count && can_reset)
                {
                    rdy_flg = true;
                }
            }
            if (can_tx && !rdy_flg || command_ask == "Init")
            {
                lss_at.SetValue("Color", Cred);
                if (command_ask == "Init")
                {
                    lss_at.SetValue("Color", Cgreen);
                }
                lss_at.Enabled = true;
                lss_at.BlinkIntervalSeconds = 0.7f;
                lss_at.BlinkLength = 20.0f;
                lss_at.Enabled = true;
                stts = "Not Ready";
            }
            if (can_tx && rdy_flg)
            {
                lss_at.SetValue("Color", Cgreen);
                lss_at.BlinkIntervalSeconds = 0;
                lss_at.BlinkLength = 10.0f;
                lss_at.Enabled = true;
                stts = "Ready";
            }
            if (!can_tx && command_ask == "Stop" || command_ask == "" || command_ask == "Freeze" || command_ask == "Eject" || command_ask == "Recall")
            {
                lss_at.SetValue("Color", Cred);
                if (command_ask == "Eject")
                {
                    lss_at.SetValue("Color", Cyellow);
                }

                if (command_ask == "Recall")
                {
                    lss_at.SetValue("Color", Cblue);
                }
                lss_at.BlinkIntervalSeconds = 0.7f;
                lss_at.BlinkLength = 20.0f;
                lss_at.Enabled = true;
                stts = "Not Ready";
            }

            if (total_drones_mining > 0 && bores_completed < t_mne_runs && can_tx && can_run || flto)
            {
                lss_at.BlinkIntervalSeconds = 0;
                if (t_dn_unk > 0)
                {
                    lss_at.BlinkIntervalSeconds = 0.7f;
                }
                lss_at.SetValue("Color", Cyellow);
                if (t_dn_dmg > 0)
                {
                    lss_at.BlinkIntervalSeconds = 0.7f;
                    lss_at.SetValue("Color", Coren);
                }
                if (flto)
                {
                    lss_at.BlinkIntervalSeconds = 0;
                    lss_at.SetValue("Color", Coren);
                }

                lss_at.Enabled = true;
                stts = "Working";
            }
            if (bores_completed >= t_mne_runs)
            {
                lss_at.SetValue("Color", Cred);
                lss_at.Enabled = true;
                stts = "Sequence Finished";
            }
            dp_txl.Clear();
            dp_txl.Append("Mining Grid Status" + " - GMDC " + ver);
            dp_txl.Append('\n');
            dp_txl.Append('\n');
            dp_txl.Append("Remaining bores: " + bores_remaining);
            dp_txl.Append('\n');

            if (ds_tg_drn.Count > 0 && drone_name.Count > 0 && ds_tg_drn[0] != null)
            {

                for (int i = 0; i < drone_name.Count; i++)
                {
                    if (rnw_hdr)
                    {
                        drninf.Clear();
                        drninf.Append("Mining Drone Status" + " - GMDC " + ver);
                        drninf.Append('\n');
                        rnw_hdr = false;
                    }


                    if (drone_name.Count > 0)
                    {
                        dps_r_d = Convert.ToDecimal(drone_name.Count) / Convert.ToDecimal(drones_per_screen);
                        double dps_r = Math.Round(Convert.ToDouble(dps_r_d), 2);
                        double sdr_c = Math.Round(Convert.ToDouble(ds_tg_drn.Count * drones_per_screen), 2);
                        if ((ds_tg_drn.Count * drones_per_screen) < drone_name.Count)
                        {
                            Echo($"Insufficient number of drone displays with tag '{dp_drn_tag}' present");
                            return;
                        }
                        if (i % 2 == 0 && i + 1 >= drone_name.Count - 1)
                        {
                            clbt = false;
                            cbval = 0;
                        }
                        if (i % 2 == 0 && i + 1 > drone_name.Count - 1)
                        {
                            clbt = false;
                            cbval = 0;
                        }
                        if (i % 2 == 0 && i + 1 <= drone_name.Count - 1)
                        {
                            clbt = true;
                            cbval = i + 1;
                        }

                        if (cbval > drone_name.Count - 1)
                        {
                            cbval = drone_name.Count - 1;
                        }
                        if (i % 2 == 0)
                        {
                            scrnbldr(i, cbval, clbt);
                        }

                        if (i == ((drones_per_screen * 1) - 1) && i <= drone_name.Count - 1 && drone_name.Count > 0 && ds_tg_drn.Count > 0 && !rnw_hdr || drone_name.Count <= drones_per_screen && i == drone_name.Count - 1)
                        {
                            if (dps_r_d > 0 && ds_tg_drn[0] != null)
                            {
                                sD = ((IMyTextSurfaceProvider)ds_tg_drn[0]).GetSurface(srfD);
                                sD.WriteText(drninf.ToString());
                            }
                            rnw_hdr = true;
                        }
                        if (i > ((drones_per_screen * 1) - 1) && i == ((drones_per_screen * 2) - 1) && i <= drone_name.Count - 1 && drone_name.Count > (drones_per_screen * 1) && ds_tg_drn.Count > 1 && !rnw_hdr || drone_name.Count <= (drones_per_screen * 2) && i == drone_name.Count - 1)
                        {
                            if (dps_r_d > 1 && ds_tg_drn[1] != null)
                            {
                                sD = ((IMyTextSurfaceProvider)ds_tg_drn[1]).GetSurface(srfD);
                                sD.WriteText(drninf.ToString());
                            }
                            rnw_hdr = true;
                        }
                        if (i > ((drones_per_screen * 2) - 1) && i == ((drones_per_screen * 3) - 1) && i <= drone_name.Count - 1 && drone_name.Count > (drones_per_screen * 2) && ds_tg_drn.Count > 1 && !rnw_hdr || drone_name.Count <= (drones_per_screen * 3) && i == drone_name.Count - 1)
                        {
                            if (dps_r_d > 2 && ds_tg_drn[2] != null)
                            {
                                sD = ((IMyTextSurfaceProvider)ds_tg_drn[2]).GetSurface(srfD);
                                sD.WriteText(drninf.ToString());
                            }
                            rnw_hdr = true;
                        }
                    }
                }
                for (int i = 0; i < grid_bore_finished.Count; i++)
                {
                    for (int j = 0; j < drone_gps_grid_list_position.Count; j++)
                    {
                        if (!grid_bore_occupied[i])
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
                            grid_bore_occupied[i] = false;
                        }
                        drone_assigns_count[j] = 0;
                    }
                    if (!grid_bore_finished[i])
                    {
                        dp_txl.Append('\n');
                        dp_txl.Append($"i: {i}  Mining: {grid_bore_occupied[i].ToString()}  Finished: {grid_bore_finished[i].ToString()} Drone: {drone_namer}");
                    }
                }
                if (ds_tg_lst.Count > 0 && ds_tg_lst[0] != null)
                {
                    sL.WriteText(dp_txl.ToString());
                }
            }

            time_count++;
            if (time_count >= t_lim)
            {
                time_delay = true;
            }
            pngt_count++;
            if (pngt_count >= p_lim)
            {
                pinged = false;
            }
            if (drones_undocking) 
            {
                undock_timer++; 
            }
            if(undock_timer > undock_delay_time)
            {
                drones_undocking = false;
            }
            Echo($"{time_count} {time_delay}");
            Echo($"{pngt_count} {pinged}");
            Echo($"{undock_timer} {drones_undocking} {total_drones_undocking}");
        }

        void GtMsgData(string data_message)
        {
            // get custom data from programmable block
            String[] msgdta = data_message.Split(':');

            //Define GPS coordinates from 
            if (msgdta.Length > 5)
            {
                recieved_drone_id_name = msgdta[0];
                if (recieved_drone_id_name.Contains(drone_tag))
                {
                    rc_dds = msgdta[1];
                    rc_tnl_end = msgdta[2];
                    rc_dn_sts = msgdta[3];
                    recieved_drone_dock = msgdta[4];
                    recieved_drone_undock = msgdta[5];
                    recived_drone_autopilot = msgdta[6];
                    rc_locx = msgdta[8];
                    rc_locy = msgdta[9];
                    rc_locz = msgdta[10];
                    rc_dn_drl_dpth = msgdta[11];
                    rc_dn_drl_crnt = msgdta[12];
                    rc_dn_drl_strt = msgdta[13];
                    rc_dn_chg = msgdta[14];
                    rc_dn_gas = msgdta[15];
                    rc_dn_str = msgdta[16];
                    if (msgdta.Length > 17)
                    {
                        rc_dn_gps_lst = msgdta[17];
                    }
                }
                else
                {
                    recieved_drone_id_name = "";
                    rc_dds = "";
                    rc_tnl_end = "";
                    rc_dn_sts = "";
                    recieved_drone_dock = "";
                    recieved_drone_undock = "";
                    recived_drone_autopilot = "";
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
                }
                if (rc_dn_gps_lst == "")
                {
                    recieved_drone_list_position = -1;
                }
                else
                {
                    if (int.TryParse(rc_dn_gps_lst, out recieved_drone_list_position))
                    {
                        int.TryParse(rc_dn_gps_lst, out recieved_drone_list_position);
                    }
                }
                if (rc_dn_chg == "")
                {
                    rc_d_cn = 0.0;
                }
                if (double.TryParse(rc_dn_chg, out rc_d_cn))
                {
                    double.TryParse(rc_dn_chg, out rc_d_cn);
                }
            }
        }

        void GtCstData()
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
                Echo("Format invalid,GPS:name:x:y:z:depth:grid:numx:numy:limit=True/False:flightfactor:flighthardlimit:skipboresnum");
                Echo("Set grid, numx, numy to 1 for single point");
                return;
            }
            if (gps_cmnd.Length > 4)
            {
                m_gps_crds = new Vector3D(Double.Parse(gps_cmnd[2]), Double.Parse(gps_cmnd[3]), Double.Parse(gps_cmnd[4]));

                cst_dt1 = gps_cmnd[1];
                cst_dt2 = gps_cmnd[2];
                cst_dt3 = gps_cmnd[3];
                cst_dt4 = gps_cmnd[4];
                cst_dt5 = gps_cmnd[5];
            }
            if (gps_cmnd.Length < 6)
            {
                cst_dt6 = "";
                drl_len = 1.0;
                cst_dt7 = "";
                grdsz = 0.0;
                cst_dt8 = "";
                nPtsX = 0;
                cst_dt9 = "";
                nPtsY = 0;
                cst_dt10 = "";
                ign_dpth = 0.0;
                cst_dt11 = "";
                launched_drone_status = false;
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 5)
            {
                if (gps_cmnd.Length > 5)
                {
                    cst_dt6 = gps_cmnd[6];
                    c_dist = cst_dt6;
                    if (Double.TryParse(c_dist, out drl_len))
                    {
                        Double.TryParse(c_dist, out drl_len);
                    }
                    else
                    {
                        drl_len = 1.0;
                    }
                }
                else
                {
                    cst_dt6 = "";
                    drl_len = 1.0;
                }
            }
            if (gps_cmnd.Length < 7)
            {
                cst_dt7 = "";
                grdsz = 0.0;
                cst_dt8 = "";
                nPtsX = 0;
                cst_dt9 = "";
                nPtsY = 0;
                cst_dt10 = "";
                ign_dpth = 0.0;
                cst_dt11 = "";
                launched_drone_status = false;
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 6)
            {
                cst_dt7 = gps_cmnd[7];
                if (double.TryParse(cst_dt7, out grdsz))
                {
                    double.TryParse(cst_dt7, out grdsz);
                }
                else
                {
                    grdsz = 0.0;
                }
            }
            else
            {
                cst_dt7 = "";
                grdsz = 0.0;
            }
            if (gps_cmnd.Length < 8)
            {
                cst_dt7 = "";
                grdsz = 0.0;
                cst_dt8 = "";
                nPtsX = 0;
                cst_dt9 = "";
                nPtsY = 0;
                cst_dt10 = "";
                ign_dpth = 0.0;
                cst_dt11 = "";
                launched_drone_status = false;
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 7)
            {
                cst_dt8 = gps_cmnd[8];
                if (int.TryParse(cst_dt8, out nPtsX))
                {
                    int.TryParse(cst_dt8, out nPtsX);
                }
                else
                {
                    nPtsX = 0;
                }
            }
            else
            {
                cst_dt8 = "";
                nPtsX = 0;
            }
            if (gps_cmnd.Length < 9)
            {
                cst_dt7 = "";
                grdsz = 0.0;
                cst_dt8 = "";
                nPtsX = 0;
                cst_dt9 = "";
                nPtsY = 0;
                cst_dt10 = "";
                ign_dpth = 0.0;
                cst_dt11 = "";
                launched_drone_status = false;
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 8)
            {
                cst_dt9 = gps_cmnd[9];

                if (int.TryParse(cst_dt9, out nPtsY))
                {
                    int.TryParse(cst_dt9, out nPtsY);
                }
                else
                {
                    nPtsY = 0;
                }
            }
            else
            {
                cst_dt9 = "";
                nPtsY = 0;
            }
            if (gps_cmnd.Length < 10)
            {
                cst_dt10 = "";
                ign_dpth = 0.0;
                cst_dt11 = "";
                launched_drone_status = false;
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 9)
            {
                cst_dt10 = gps_cmnd[10];
                if (Double.TryParse(cst_dt10, out ign_dpth))
                {
                    Double.TryParse(cst_dt10, out ign_dpth);
                }
                else
                {
                    ign_dpth = 0.0;
                }
            }
            else
            {
                cst_dt10 = "";
                ign_dpth = 0.0;
            }
            if (gps_cmnd.Length < 12)
            {
                cst_dt11 = "";
                launched_drone_status = false;
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 11)
            {
                cst_dt11 = gps_cmnd[11];
                if (bool.TryParse(cst_dt11, out launched_drone_status))
                {
                    bool.TryParse(cst_dt11, out launched_drone_status);
                }
                else
                {
                    launched_drone_status = false;
                }
            }
            else
            {
                cst_dt11 = "";
                launched_drone_status = false;
            }
            if (gps_cmnd.Length < 13)
            {
                cst_dt12 = "";
                ft_ftr = 1;
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 12)
            {
                cst_dt12 = gps_cmnd[12];
                if (int.TryParse(cst_dt12, out ft_ftr))
                {
                    int.TryParse(cst_dt12, out ft_ftr);
                }
                else
                {
                    ft_ftr = 1;
                }
            }
            else
            {
                cst_dt12 = "";
                ft_ftr = 1;
            }
            if (gps_cmnd.Length < 14)
            {
                cst_dt13 = "";
                hd_lm = 6;
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 13)
            {
                cst_dt13 = gps_cmnd[13];
                if (int.TryParse(cst_dt13, out hd_lm))
                {
                    int.TryParse(cst_dt13, out hd_lm);
                }
                else
                {
                    hd_lm = 6;
                }
            }
            else
            {
                cst_dt13 = "";
                hd_lm = 6;
            }
            if (gps_cmnd.Length < 15)
            {
                cst_dt14 = "";
                skp_br = 0;
                return;
            }
            if (gps_cmnd.Length > 14)
            {
                cst_dt14 = gps_cmnd[14];
                if (int.TryParse(cst_dt14, out skp_br))
                {
                    int.TryParse(cst_dt14, out skp_br);
                }
                else
                {
                    skp_br = 0;
                }
            }
            else
            {
                cst_dt14 = "";
                skp_br = 0;
            }
        }
        void GtRCData()
        {
            String[] rem_gps_cmd = rm_ctl_act.CustomData.Split(':');
            if (rem_gps_cmd.Length < 6)
            {
                rm_cst_dat1 = "";
                rm_cst_dat2 = "";
                rm_cst_dat3 = "";
                rm_cst_dat4 = "";
                rm_cst_dat5 = "";
                rm_cst_dat6 = "";
                tgt_vld = true;
                return;
            }
            if (rem_gps_cmd.Length > 6)
            {
                t_gps_crds = new Vector3D(Double.Parse(rem_gps_cmd[2]), Double.Parse(rem_gps_cmd[3]), Double.Parse(rem_gps_cmd[4]));
                tgt_vld = true;
                rm_cst_dat1 = rem_gps_cmd[1];
                rm_cst_dat2 = rem_gps_cmd[2];
                rm_cst_dat3 = rem_gps_cmd[3];
                rm_cst_dat4 = rem_gps_cmd[4];
                rm_cst_dat5 = rem_gps_cmd[5];
                rm_cst_dat6 = rem_gps_cmd[6];
                if (double.TryParse(rm_cst_dat6, out safe_dstvl))
                {
                    double.TryParse(rm_cst_dat6, out safe_dstvl);
                }
                else
                {
                    safe_dstvl = 0.0;
                }
            }
            if (rem_gps_cmd.Length < 12 && rem_gps_cmd.Length > 7)
            {
                rm_cst_dat7 = "";
                rm_cst_dat8 = "";
                rm_cst_dat9 = "";
                rm_cst_dat10 = "";
                rm_cst_dat11 = "";
                rm_cst_dat12 = "";
                astd_vld = false;
                return;
            }
            if (rem_gps_cmd.Length > 7)
            {
                a_gps_crds = new Vector3D(Double.Parse(rem_gps_cmd[9]), Double.Parse(rem_gps_cmd[10]), Double.Parse(rem_gps_cmd[11]));
                astd_vld = true;
                rm_cst_dat7 = rem_gps_cmd[7];
                rm_cst_dat8 = rem_gps_cmd[8];
                rm_cst_dat9 = rem_gps_cmd[9];
                rm_cst_dat10 = rem_gps_cmd[10];
                rm_cst_dat11 = rem_gps_cmd[11];
                rm_cst_dat12 = rem_gps_cmd[12];
            }
        }
        public List<Vector3D> GenGrdPosits(Vector3D centerPoint, Vector3D planeNormal, double gridSize, int numPointsX, int numPointsY)
        {
            List<Vector3D> grdPositins = new List<Vector3D>();
            dspl = new List<Vector3D>();
            grid_bore_finished.Clear();
            grid_bore_occupied.Clear();
            grid_bore_finished = new List<bool>();
            grid_bore_occupied = new List<bool>();
            Vector3D xAxis = Vector3D.CalculatePerpendicularVector(planeNormal);
            Vector3D yAxis = Vector3D.Cross(planeNormal, xAxis);
            Vector3D halfOffsetX = (numPointsX - 1) * 0.5 * gridSize * xAxis;
            Vector3D halfOffsetY = (numPointsY - 1) * 0.5 * gridSize * yAxis;
            for (int i = 0; i < numPointsX; i++)
            {
                for (int j = 0; j < numPointsY; j++)
                {
                    Vector3D position = centerPoint + i * gridSize * xAxis - j * gridSize * yAxis - halfOffsetX + halfOffsetY;
                    Vector3D disp_cent = (centerPoint - position);
                    grdPositins.Add(position);
                    dspl.Add(disp_cent);
                    grid_bore_occupied.Add(false);
                    grid_bore_finished.Add(false);
                }
            }
            return grdPositins;
        }
        public void CyclNextCord()
        {
            current_gps_idx = (current_gps_idx + 1) % grid_bore_positions.Count;
            next_gps_crds = grid_bore_positions[current_gps_idx];
        }
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

        public void scrnbldr(int ivl, int ivl2, bool slu)
        {
            cl[0] = drone_name[ivl] + " Status: " + drone_damage_state[ivl] + " " + drone_control_status[ivl];
            cl[1] = drone_name[ivl] + (" Docked: ") + drone_dock_status[ivl];
            cl[2] = drone_name[ivl] + (" Undocked: ") + drone_undock_status[ivl];
            cl[3] = drone_name[ivl] + (" Finished: ") + drone_tunnel_complete[ivl];
            cl[4] = drone_name[ivl] + (" Mining: ") + drone_mining[ivl];
            cl[5] = drone_name[ivl] + " Waiting: " + drone_must_wait[ivl] + " Reset: " + drone_reset_func[ivl];
            cl[6] = "Charge: " + drone_charge_storage[ivl] + "%" + " Tank: " + drone_gas_storage[ivl] + "%" + " Cargo: " + drone_ore_storage[ivl] + "%";
            cl[7] = "Drill depth: " + drone_drill_depth_value[ivl] + "m" + " Start: " + drone_mine_depth_start_status[ivl] + "m";
            cl[8] = "Current depth: " + drone_mine_distance_status[ivl] + "m";
            cl[9] = "Drone control seq: " + drone_control_sequence[ivl] + "  Recall seq: " + drone_recall_sequence[ivl] + " " + drone_recall_list[ivl];
            cl[10] = "Location: " + " " + drone_gps_grid_list_position[ivl] + "  Asnd: " + drone_assigned_coordinates[ivl] + " Unit OK: " + dst[ivl];
            cl[11] = "X: " + drone_location_x[ivl] + " Y: " + drone_location_y[ivl] + " Z: " + drone_location_z[ivl];
            if (slu)
            {
                cl2[0] = drone_name[ivl2] + " Status: " + drone_damage_state[ivl2] + " " + drone_control_status[ivl2];
                cl2[1] = drone_name[ivl2] + (" Docked: ") + drone_dock_status[ivl2];
                cl2[2] = drone_name[ivl2] + (" Undocked: ") + drone_undock_status[ivl2];
                cl2[3] = drone_name[ivl2] + (" Finished: ") + drone_tunnel_complete[ivl2];
                cl2[4] = drone_name[ivl2] + (" Mining: ") + drone_mining[ivl2];
                cl2[5] = drone_name[ivl2] + " Waiting: " + drone_must_wait[ivl2] + " Reset: " + drone_reset_func[ivl2];
                cl2[6] = "Charge: " + drone_charge_storage[ivl2] + "%" + " Tank: " + drone_gas_storage[ivl2] + "%" + " Cargo: " + drone_ore_storage[ivl2] + "%";
                cl2[7] = "Drill depth: " + drone_drill_depth_value[ivl2] + "m" + " Start: " + drone_mine_depth_start_status[ivl2] + "m";
                cl2[8] = "Current depth: " + drone_mine_distance_status[ivl2] + "m";
                cl2[9] = "Drone control seq: " + drone_control_sequence[ivl2] + "  Recall seq: " + drone_recall_sequence[ivl2] + " " + drone_recall_list[ivl2];
                cl2[10] = "Loc: " + " " + drone_gps_grid_list_position[ivl2] + "  Asnd: " + drone_assigned_coordinates[ivl2] + " Unit OK: " + dst[ivl2];
                cl2[11] = "X: " + drone_location_x[ivl2] + " Y: " + drone_location_y[ivl2] + " Z: " + drone_location_z[ivl2];
            }
            for (int i = 0; i < cl.Count; i++)
            {
                tla[i] = cl[i].Length;
                rst[i] = clbs - tla[i];
                jxt.Clear();
                for (int j = 0; j < rst[i]; j++)
                {
                    jxt.Append(" ");
                }
                fct[i] = jxt.ToString();
            }

            drninf.Append('\n');
            for (int i = 0; i < cl.Count; i++)
            {
                drninf.Append(cl[i] + fct[i] + cl2[i]);
                drninf.Append('\n');
            }
        }
        public void cbld(string cdata_1, string xpos, string ypos, string zpos, string cdata_5, string cmdo, string data_6, string idepth, string xpos2, string ypos2, string zpos2)
        {
            c.Clear();
            c.Append("GPS" + ":" + cdata_1 + ":" + xpos + ":" + ypos + ":" + zpos + ":" + cdata_5 + ":" + cmdo + ":" + data_6 + ":" + idepth + ":");
            if (astd_vld)
            {
                c.Append("GPS" + ":" + "PAD" + ":" + xpos2 + ":" + ypos2 + ":" + zpos2 + ":" + "#FF75C9F1" + ":");
            }
            return;
        }

        void GtStrD()
        {
            if (Storage != null)
            {
                String[] str_data = Storage.Split(';');
                if (str_data.Length > 0)
                {
                    for (int i = 0; i < str_data.Length - 1; i++)
                    {
                        String[] str_datai = str_data[i].Split(':');
                        if (str_datai.Length > 0)
                        {
                            int bn;
                            int bc;
                            if (int.TryParse(str_datai[0], out bn))
                            {
                                int.TryParse(str_datai[0], out bn);
                                if (bn > 0)
                                {
                                    grid_bore_finished.Add(true);
                                }
                                else
                                {
                                    grid_bore_finished.Add(false);
                                }
                            }
                            if (int.TryParse(str_datai[1], out bc))
                            {
                                int.TryParse(str_datai[1], out bc);
                                if (bc > 0)
                                {
                                    grid_bore_occupied.Add(true);
                                }
                                else
                                {
                                    grid_bore_occupied.Add(false);
                                }
                            }
                        }
                    }
                }
            }
        }
        void dtxt()
        {
            IGC.SendBroadcastMessage(tx_chan, dt_out[di], TransmissionDistance.TransmissionDistanceMax);
        }
        //program end

    }
}
