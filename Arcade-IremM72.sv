//============================================================================
//  Irem M72 for MiSTer FPGA
//
//  Copyright (C) 2022 Martin Donlon
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================


import m72_pkg::*;

module emu
(
    //Master input clock
    input         CLK_50M,

    //Async reset from top-level module.
    //Can be used as initial reset.
    input         RESET,

    //Must be passed to hps_io module
    inout  [48:0] HPS_BUS,

    //Base video clock. Usually equals to CLK_SYS.
    output        CLK_VIDEO,

    //Multiple resolutions are supported using different CE_PIXEL rates.
    //Must be based on CLK_VIDEO
    output        CE_PIXEL,

    //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
    //if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
    output [12:0] VIDEO_ARX,
    output [12:0] VIDEO_ARY,

    output  [7:0] VGA_R,
    output  [7:0] VGA_G,
    output  [7:0] VGA_B,
    output        VGA_HS,
    output        VGA_VS,
    output        VGA_DE,    // = ~(VBlank | HBlank)
    output        VGA_F1,
    output [1:0]  VGA_SL,
    output        VGA_SCALER, // Force VGA scaler

    input  [11:0] HDMI_WIDTH,
    input  [11:0] HDMI_HEIGHT,
    output        HDMI_FREEZE,

`ifdef MISTER_FB
    // Use framebuffer in DDRAM (USE_FB=1 in qsf)
    // FB_FORMAT:
    //    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
    //    [3]   : 0=16bits 565 1=16bits 1555
    //    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
    //
    // FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
    output        FB_EN,
    output  [4:0] FB_FORMAT,
    output [11:0] FB_WIDTH,
    output [11:0] FB_HEIGHT,
    output [31:0] FB_BASE,
    output [13:0] FB_STRIDE,
    input         FB_VBL,
    input         FB_LL,
    output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
    // Palette control for 8bit modes.
    // Ignored for other video modes.
    output        FB_PAL_CLK,
    output  [7:0] FB_PAL_ADDR,
    output [23:0] FB_PAL_DOUT,
    input  [23:0] FB_PAL_DIN,
    output        FB_PAL_WR,
`endif
`endif

    output        LED_USER,  // 1 - ON, 0 - OFF.

    // b[1]: 0 - LED status is system status OR'd with b[0]
    //       1 - LED status is controled solely by b[0]
    // hint: supply 2'b00 to let the system control the LED.
    output  [1:0] LED_POWER,
    output  [1:0] LED_DISK,

    // I/O board button press simulation (active high)
    // b[1]: user button
    // b[0]: osd button
    output  [1:0] BUTTONS,

    input         CLK_AUDIO, // 24.576 MHz
    output [15:0] AUDIO_L,
    output [15:0] AUDIO_R,
    output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
    output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

    //ADC
    inout   [3:0] ADC_BUS,

    //SD-SPI
    output        SD_SCK,
    output        SD_MOSI,
    input         SD_MISO,
    output        SD_CS,
    input         SD_CD,

    //High latency DDR3 RAM interface
    //Use for non-critical time purposes
    output        DDRAM_CLK,
    input         DDRAM_BUSY,
    output  [7:0] DDRAM_BURSTCNT,
    output [28:0] DDRAM_ADDR,
    input  [63:0] DDRAM_DOUT,
    input         DDRAM_DOUT_READY,
    output        DDRAM_RD,
    output [63:0] DDRAM_DIN,
    output  [7:0] DDRAM_BE,
    output        DDRAM_WE,

    //SDRAM interface with lower latency
    output        SDRAM_CLK,
    output        SDRAM_CKE,
    output [12:0] SDRAM_A,
    output  [1:0] SDRAM_BA,
    inout  [15:0] SDRAM_DQ,
    output        SDRAM_DQML,
    output        SDRAM_DQMH,
    output        SDRAM_nCS,
    output        SDRAM_nCAS,
    output        SDRAM_nRAS,
    output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
    //Secondary SDRAM
    //Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
    input         SDRAM2_EN,
    output        SDRAM2_CLK,
    output [12:0] SDRAM2_A,
    output  [1:0] SDRAM2_BA,
    inout  [15:0] SDRAM2_DQ,
    output        SDRAM2_nCS,
    output        SDRAM2_nCAS,
    output        SDRAM2_nRAS,
    output        SDRAM2_nWE,
`endif

    input         UART_CTS,
    output        UART_RTS,
    input         UART_RXD,
    output        UART_TXD,
    output        UART_DTR,
    input         UART_DSR,

    // Open-drain User port.
    // 0 - D+/RX
    // 1 - D-/TX
    // 2..6 - USR2..USR6
    // Set USER_OUT to 1 to read from USER_IN.
    input   [6:0] USER_IN,
    output  [6:0] USER_OUT,

    input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;

//LLAPI
//Done later from the LLAPI main block as USER_OUT is now an option with LLAPI available
//assign USER_OUT  = '1;
//END

assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign CLK_VIDEO = CLK_32M;

assign VGA_F1 = 0;
assign VGA_SCALER = 0;

assign AUDIO_S = 1;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;

//assign BUTTONS = 0;
//LLAPI
//Remap OSD buttons to LLAPI specifc combinaison (see LLAPI main block)
assign BUTTONS   = llapi_osd;
//END

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[2:1];
wire [1:0] scandoubler_fx = status[4:3];
wire [1:0] scale = status[6:5];
wire pause_in_osd = status[7];
wire system_pause;

assign VGA_SL = scandoubler_fx;
assign HDMI_FREEZE = 0; //system_pause;

wire en_layer_a = ~status[64];
wire en_layer_b = ~status[65];
wire en_sprites = ~status[66];
wire en_layer_palette = ~status[67];
wire en_sprite_palette = ~status[68];
wire dbg_sprite_freeze = status[69];
wire en_audio_filters = ~status[70];

wire video_60hz = status[9:8] == 2'd3;
wire video_57hz = status[9:8] == 2'd2;
wire video_50hz = status[9:8] == 2'd1;

// If video timing changes, force mode update
reg [1:0] video_status;
reg new_vmode = 0;
always @(posedge clk_sys) begin
    if (video_status != status[9:8]) begin
        video_status <= status[9:8];
        new_vmode <= ~new_vmode;
    end
end

`include "build_id.v" 
localparam CONF_STR = {
    "M72;;",
    "-;",
    "P1,Video Settings;",
    "P1O[2:1],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
    "P1O[4:3],Scandoubler Fx,None,CRT 25%,CRT 50%,CRT 75%;",
    "P1O[6:5],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
    "P1-;",
    "P1O[9:8],Video Timing,Normal(55Hz),50Hz,57Hz,60Hz;",
    "P1O[10],Orientation,Horz,Vert;",
    "P1-;",
    "d1P1O[11],240p Crop,Off,On;",
    "d2P1O[16:12],Crop Offset,0,1,2,3,4,5,6,7,8,-8,-7,-6,-5,-4,-3,-2,-1;",
    "P1-;",
    "P1O[20:17],Analog Video H-Pos,0,-1,-2,-3,-4,-5,-6,-7,8,7,6,5,4,3,2,1;",
	"P1O[24:21],Analog Video V-Pos,0,-1,-2,-3,-4,-5,-6,-7,8,7,6,5,4,3,2,1;",
    "-;",
    "O[7],OSD Pause,Off,On;",

    //LLAPI
    //Add LLAPI option to the OSD menu
    //Need to reserve 1 bit :  "OM" = status[22] as there are 2 options (None, LLAPI) 
    //To detect LLAPI status[22] should be TRUE
    //None      : 00
    //LLAPI     : 01
    //Always double check witht the bits map allocation table to avoid conflicts    "-;",
    "OM,BlisSTer Mode,Off,ON;",
    //END

    "-;",
    "DIP;",
    "-;",
    "P2,Debug;",
    "P2-;",
    "P2O[64],Layer A,On,Off;",
    "P2O[65],Layer B,On,Off;",
    "P2O[66],Sprites,On,Off;",
    "P2O[67],Layer Palette,On,Off;",
    "P2O[68],Sprite Palette,On,Off;",
    "P2O[69],Sprite Freeze,Off,On;",
    "P2O[70],Audio Filtering,On,Off;",
    "-;",
    "T[0],Reset;",
    "DEFMRA,/_Arcade/m72.mra;",
    "V,v",`BUILD_DATE 
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [128:0] status;
wire [10:0] ps2_key;

wire        ioctl_download;
wire        ioctl_upload;
wire        ioctl_upload_req = 0;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire  [7:0] ioctl_din = 0;
wire        ioctl_wait;

wire [15:0] joystick_0, joystick_1;
wire [15:0] joy = joystick_0 | joystick_1;

//LLAPI
//removed default allocation as we now have 2 options USB and LLAPI
wire [15:0] joy1;
wire [15:0] joy2; 
//END

wire [15:0] joy1a;
wire [15:0] joy2a;

wire [21:0] gamma_bus;
wire        direct_video;
wire        video_rotated;
wire        no_rotate = ~status[10];
wire        flip = 0;
wire        rotate_ccw = 1;

wire        allow_crop_240p = ~forced_scandoubler && scale == 0;
wire        crop_240p = allow_crop_240p & status[11];
wire [4:0]  crop_offset = status[16:12] < 9 ? {status[16:12]} : ( status[16:12] + 5'd15 );


wire clk_sys = CLK_32M;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
    .clk_sys(clk_sys),
    .HPS_BUS(HPS_BUS),
    .EXT_BUS(),
    .gamma_bus(gamma_bus),
    .direct_video(direct_video),

    .forced_scandoubler(forced_scandoubler),
    .new_vmode(new_vmode),
    .video_rotated(video_rotated),

    .buttons(buttons),
    .status(status),
    .status_menumask({crop_240p, allow_crop_240p, direct_video}),

    .ioctl_download(ioctl_download),
    .ioctl_upload(ioctl_upload),
    .ioctl_upload_req(ioctl_upload_req),
    .ioctl_wr(ioctl_wr),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout),
    .ioctl_din(ioctl_din),
    .ioctl_index(ioctl_index),
    .ioctl_wait(ioctl_wait),

    .joystick_0(joystick_0),
    .joystick_1(joystick_1),
    .ps2_key(ps2_key)
);

//////////////////   LLAPI   ///////////////////

reg llapi_button_pressed, llapi_button_pressed2;

//Initialise BliSTer port 1 and port 2 based on first button pressed
always @(posedge CLK_50M) begin
        if (reset) begin
                llapi_button_pressed  <= 0;
                llapi_button_pressed2 <= 0;
        end else begin
                if (|llapi_buttons)
                        llapi_button_pressed  <= 1;
                if (|llapi_buttons2)
                        llapi_button_pressed2 <= 1;
        end
end

// controller id is 0 if there is either an Atari controller or no controller
// if id is 0, assume there is no controller until a button is pressed
// also check for 255 and treat that as 'no controller' as well
wire use_llapi  = llapi_en  && llapi_select && ((|llapi_type  && ~(&llapi_type))  || llapi_button_pressed);
wire use_llapi2 = llapi_en2 && llapi_select && ((|llapi_type2 && ~(&llapi_type2)) || llapi_button_pressed2);

wire [31:0] llapi_buttons, llapi_buttons2;
wire [71:0] llapi_analog, llapi_analog2;
wire [7:0]  llapi_type, llapi_type2;
wire llapi_en, llapi_en2;

wire llapi_select = status[22]; // This is the status bit from the Menu (see Menu configuration block to check what bits need to be tested)

wire llapi_latch_o, llapi_latch_o2, llapi_data_o, llapi_data_o2;

//connect the pins of USER I/O port from the I/O board to BliSTer (if LLAPI has been selected in the OSD menu)

// Indexes for reference:
// 0 = D+    = P1 Latch
// 1 = D-    = P1 Data
// 2 = TX-   = LLAPI Enable
// 3 = GND_d = N/C
// 4 = RX+   = P2 Latch
// 5 = RX-   = P2 Data

always_comb begin
    USER_OUT = 6'b111111;
    //LLAPI selected in OSD menu
    if (llapi_select) begin
        USER_OUT[0] = llapi_latch_o;
        USER_OUT[1] = llapi_data_o;
        USER_OUT[2] = ~(llapi_select & ~OSD_STATUS); //This is the Red or Green LED on the BliSter
        USER_OUT[4] = llapi_latch_o2;
        USER_OUT[5] = llapi_data_o2;
    end else begin
        USER_OUT[0] = 1'b1;
        USER_OUT[1] = 1'b1;
    end
end

//LLAPI string configuration for port 1
LLAPI llapi
(
    .CLK_50M(CLK_50M),
    .LLAPI_SYNC(video_vs),
    .IO_LATCH_IN(USER_IN[0]),
    .IO_LATCH_OUT(llapi_latch_o),
    .IO_DATA_IN(USER_IN[1]),
    .IO_DATA_OUT(llapi_data_o),
    .ENABLE(llapi_select & ~OSD_STATUS),
    .LLAPI_BUTTONS(llapi_buttons),
    .LLAPI_ANALOG(llapi_analog),
    .LLAPI_TYPE(llapi_type),
    .LLAPI_EN(llapi_en)
);

//LLAPI string configuration for port 2
LLAPI llapi2
(
    .CLK_50M(CLK_50M),
    .LLAPI_SYNC(video_vs),
    .IO_LATCH_IN(USER_IN[4]),
    .IO_LATCH_OUT(llapi_latch_o2),
    .IO_DATA_IN(USER_IN[5]),
    .IO_DATA_OUT(llapi_data_o2),
    .ENABLE(llapi_select & ~OSD_STATUS),
    .LLAPI_BUTTONS(llapi_buttons2),
    .LLAPI_ANALOG(llapi_analog2),
    .LLAPI_TYPE(llapi_type2),
    .LLAPI_EN(llapi_en2)
);


// Controller string provided by core for reference (order is important)
//Controller specific mapping based on type. More info here : https://docs.google.com/document/d/12XpxrmKYx_jgfEPyw-O2zex1kTQZZ-NSBdLO2RQPRzM/edit
//To be checked : button ref id are HID button id ?

//Mapping :     "J1,Dig Left,Dig Right,Start 1P,Start 2P,Coin,Pause;",

//P1
wire [15:0] joy_ll_a = { 8'd0,                                                 // Pause
    llapi_buttons[4],  llapi_buttons[6],  llapi_buttons[5],  llapi_buttons[3], llapi_buttons[2], llapi_buttons[1], llapi_buttons[0], // Coin Start-2P Start-1P Dig right Dig Left
    llapi_buttons[27], llapi_buttons[26], llapi_buttons[25], llapi_buttons[24] // d-pad
};

//P2
wire [15:0] joy_ll_b = { 8'd0,                                                 // Pause
    llapi_buttons2[4],  llapi_buttons2[6],  llapi_buttons2[5], llapi_buttons2[3], llapi_buttons2[2], llapi_buttons2[1], llapi_buttons2[0], // Coin Start-2P Start-1P Dig right Dig Left
    llapi_buttons2[27], llapi_buttons2[26], llapi_buttons2[25], llapi_buttons2[24] // d-pad
};

//Assign (DOWN + START + FIRST BUTTON) Combinaison to bring the OSD up - P1 and P1 ports
wire llapi_osd = (llapi_buttons[26] & llapi_buttons[5] & llapi_buttons[0]) || (llapi_buttons2[26] & llapi_buttons2[5] & llapi_buttons2[0]);


//It looks this part is a little bit core specifc. Need to check if that could not be standardized
// if LLAPI is enabled, shift USB controllers over to the next available player slot
// This basically connect the console ports to the USB joysticks or to the to LLAPI ones (that have been created above)

always_comb begin
        if (use_llapi & use_llapi2) begin
                joy1 = joy_ll_a;
                joy2 = joy_ll_b;
        end else if (use_llapi ^ use_llapi2) begin
                joy1 = use_llapi  ? joy_ll_a : joy1a;
                joy2 = use_llapi2 ? joy_ll_b : joy1a;
        end else begin
                joy1 = joy1a;
                joy2 = joy2a;
        end
end

////////////////     END OF LLAPI MAIN BLOCK   ///////////////////////////////////////////////////////////

///////////////////////   CLOCKS   ///////////////////////////////

wire CLK_32M;
wire CLK_96M;
wire pll_locked;
pll pll
(
    .refclk(CLK_50M),
    .rst(0),
    .outclk_0(CLK_96M),
    .outclk_1(CLK_32M),
    .locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1];

///////////////////////////////////////////////////////////////////////
// SDRAM
///////////////////////////////////////////////////////////////////////
wire [63:0] sdr_sprite_dout;
wire [24:0] sdr_sprite_addr;
wire sdr_sprite_req, sdr_sprite_rdy;

wire [31:0] sdr_bg_dout;
wire [24:0] sdr_bg_addr;
wire sdr_bg_req, sdr_bg_rdy;

wire [15:0] sdr_cpu_dout, sdr_cpu_din;
wire [24:0] sdr_cpu_addr;
wire sdr_cpu_req;
wire [1:0] sdr_cpu_wr_sel;

reg [24:0] sdr_rom_addr;
reg [15:0] sdr_rom_data;
reg [1:0] sdr_rom_be;
reg sdr_rom_req;

wire sdr_rom_write = ioctl_download && (ioctl_index == 0);
wire [24:0] sdr_ch3_addr = sdr_rom_write ? sdr_rom_addr : sdr_cpu_addr;
wire [15:0] sdr_ch3_din = sdr_rom_write ? sdr_rom_data : sdr_cpu_din;
wire [1:0] sdr_ch3_be = sdr_rom_write ? sdr_rom_be : sdr_cpu_wr_sel;
wire sdr_ch3_rnw = sdr_rom_write ? 1'b0 : ~{|sdr_cpu_wr_sel};
wire sdr_ch3_req = sdr_rom_write ? sdr_rom_req : sdr_cpu_req;
wire sdr_ch3_rdy;
wire sdr_cpu_rdy = sdr_ch3_rdy;
wire sdr_rom_rdy = sdr_ch3_rdy;

wire [19:0] bram_addr;
wire [7:0] bram_data;
wire [4:0] bram_cs;
wire bram_wr;

board_cfg_t board_cfg;

sdram sdram
(
    .*,
    .doRefresh(0),
    .init(~pll_locked),
    .clk(CLK_96M),

    .ch1_addr(sdr_bg_addr[24:1]),
    .ch1_dout(sdr_bg_dout),
    .ch1_req(sdr_bg_req),
    .ch1_ready(sdr_bg_rdy),

    .ch2_addr(sdr_sprite_addr[24:1]),
    .ch2_dout(sdr_sprite_dout),
    .ch2_req(sdr_sprite_req),
    .ch2_ready(sdr_sprite_rdy),

    // multiplexed with rom download and cpu read/writes
    .ch3_addr(sdr_ch3_addr[24:1]),
    .ch3_din(sdr_ch3_din),
    .ch3_dout(sdr_cpu_dout),
    .ch3_be(sdr_ch3_be),
    .ch3_rnw(sdr_ch3_rnw),
    .ch3_req(sdr_ch3_req),
    .ch3_ready(sdr_ch3_rdy)
);

rom_loader rom_loader(
    .sys_clk(clk_sys),
    .ram_clk(CLK_96M),

    .ioctl_wr(ioctl_wr && !ioctl_index),
    .ioctl_data(ioctl_dout[7:0]),

    .ioctl_wait(ioctl_wait),

    .sdr_addr(sdr_rom_addr),
    .sdr_data(sdr_rom_data),
    .sdr_be(sdr_rom_be),
    .sdr_req(sdr_rom_req),
    .sdr_rdy(sdr_rom_rdy),

    .bram_addr(bram_addr),
    .bram_data(bram_data),
    .bram_cs(bram_cs),
    .bram_wr(bram_wr),

    .board_cfg(board_cfg)
);


///////////////////         Keyboard           //////////////////
reg btn_up       = 0;
reg btn_down     = 0;
reg btn_left     = 0;
reg btn_right    = 0;
reg btn_a        = 0;
reg btn_b        = 0;
reg btn_x        = 0;
reg btn_y        = 0;
reg btn_coin1    = 0;
reg btn_coin2    = 0;
reg btn_1p_start = 0;
reg btn_2p_start = 0;
reg btn_pause    = 0;

wire pressed = ps2_key[9];
wire [7:0] code = ps2_key[7:0];
always @(posedge CLK_32M) begin
    reg old_state;
    old_state <= ps2_key[10];
    if(old_state != ps2_key[10]) begin
        case(code)
            'h16: btn_1p_start <= pressed; // 1
            'h1E: btn_2p_start <= pressed; // 2
            'h2E: btn_coin1    <= pressed; // 5
            'h36: btn_coin2    <= pressed; // 6
            'h4D: btn_pause    <= pressed; // P

            'h75: btn_up      <= pressed; // up
            'h72: btn_down    <= pressed; // down
            'h6B: btn_left    <= pressed; // left
            'h74: btn_right   <= pressed; // right
            'h14: btn_a       <= pressed; // ctrl
            'h11: btn_b       <= pressed; // alt
            'h29: btn_x       <= pressed; // space
            'h12: btn_y       <= pressed; // shift
        endcase
    end
end

// DIP SWITCHES
reg [7:0] dip_sw[8];	// Active-LOW
always @(posedge CLK_32M) begin
    if(ioctl_wr && (ioctl_index==254) && !ioctl_addr[24:3])
        dip_sw[ioctl_addr[2:0]] <= ioctl_dout;
end


// ORIGINAL - Arcade Buttons/Interfaces   ///////////////////////////

//Player 1
//wire m_up1      = btn_up      | joystick_0[3];
//wire m_down1    = btn_down    | joystick_0[2];
//wire m_left1    = btn_left    | joystick_0[1];
//wire m_right1   = btn_right   | joystick_0[0];
//wire m_btna1    = btn_a       | joystick_0[4];
//wire m_btnb1    = btn_b       | joystick_0[5];
//wire m_btnx1    = btn_x       | joystick_0[6];
//wire m_btny1    = btn_y       | joystick_0[7];

//Player 2
//wire m_up2      = btn_up      | joystick_1[3];
//wire m_down2    = btn_down    | joystick_1[2];
//wire m_left2    = btn_left    | joystick_1[1];
//wire m_right2   = btn_right   | joystick_1[0];
//wire m_btna2    = btn_a       | joystick_1[4];
//wire m_btnb2    = btn_b       | joystick_1[5];
//wire m_btnx2    = btn_x       | joystick_1[6];
//wire m_btny2    = btn_y       | joystick_1[7];

//Start/coin
//wire m_start1   = btn_1p_start | joy[8];
//wire m_start2   = btn_2p_start | joy[10];
//wire m_coin1    = btn_coin1    | joy[9];
//wire m_coin2    = btn_coin2;
//wire m_pause    = btn_pause    | joy[11];

// LLAPI - Arcade Buttons/Interfaces   ///////////////////////////

//Player 1
wire m_up1      = btn_up      | joy1[3];
wire m_down1    = btn_down    | joy1[2];
wire m_left1    = btn_left    | joy1[1];
wire m_right1   = btn_right   | joy1[0];
wire m_btna1    = btn_a       | joy1[4];
wire m_btnb1    = btn_b       | joy1[5];
wire m_btnx1    = btn_x       | joy1[6];
wire m_btny1    = btn_y       | joy1[7];

//Player 2
wire m_up2      = btn_up      | joy2[3];
wire m_down2    = btn_down    | joy2[2];
wire m_left2    = btn_left    | joy2[1];
wire m_right2   = btn_right   | joy2[0];
wire m_btna2    = btn_a       | joy2[4];
wire m_btnb2    = btn_b       | joy2[5];
wire m_btnx2    = btn_x       | joy2[6];
wire m_btny2    = btn_y       | joy2[7];

//Start/coin
wire m_start1   = btn_1p_start | joy1[8];
wire m_start2   = btn_2p_start | joy[10];
wire m_coin1    = btn_coin1    | joy1[9];
wire m_coin2    = btn_coin2;
wire m_pause    = btn_pause    | joy1[11];

//////////////////////////////////////////////////////////////////

wire [7:0] R, G, B;
wire HBlank, VBlank, HSync, VSync, hs_core, vs_core;
wire ce_pix;

m72 m72(
    .CLK_32M(CLK_32M),
    .CLK_96M(CLK_96M),
    .ce_pix(ce_pix),
    .reset_n(~reset),
    .HBlank(HBlank),
    .VBlank(VBlank),
    .HSync(hs_core),
    .VSync(vs_core),
    .R(R),
    .G(G),
    .B(B),
    .AUDIO_L(AUDIO_L),
    .AUDIO_R(AUDIO_R),

    .board_cfg(board_cfg),

    .coin({~m_coin2, ~m_coin1}),
    
    .start_buttons({~m_start2, ~m_start1}),
    
    .p1_joystick({~m_up1, ~m_down1, ~m_left1, ~m_right1}),
    .p2_joystick({~m_up2, ~m_down2, ~m_left2, ~m_right2}),
    .p1_buttons({~m_btna1, ~m_btnb1, ~m_btnx1, ~m_btny1}),
    .p2_buttons({~m_btna2, ~m_btnb2, ~m_btnx2, ~m_btny2}),
    
    .dip_sw({~dip_sw[1], ~dip_sw[0]}),

    .sdr_sprite_addr(sdr_sprite_addr),
    .sdr_sprite_dout(sdr_sprite_dout),
    .sdr_sprite_req(sdr_sprite_req),
    .sdr_sprite_rdy(sdr_sprite_rdy),

    .sdr_bg_addr(sdr_bg_addr),
    .sdr_bg_dout(sdr_bg_dout),
    .sdr_bg_req(sdr_bg_req),
    .sdr_bg_rdy(sdr_bg_rdy),

    .sdr_cpu_dout(sdr_cpu_dout),
    .sdr_cpu_din(sdr_cpu_din),
    .sdr_cpu_addr(sdr_cpu_addr),
    .sdr_cpu_req(sdr_cpu_req),
    .sdr_cpu_rdy(sdr_cpu_rdy),
    .sdr_cpu_wr_sel(sdr_cpu_wr_sel),

    .clk_bram(clk_sys),
    .bram_addr(bram_addr),
    .bram_data(bram_data),
    .bram_cs(bram_cs),
    .bram_wr(bram_wr),

`ifdef M72_DEBUG
    .pause_rq(system_pause | debug_stall),
`else
    .pause_rq(system_pause),
`endif
    .ddr_debug_data(ddr_debug_data),
    
    .en_layer_a(en_layer_a),
    .en_layer_b(en_layer_b),
    .en_sprites(en_sprites),
    .en_layer_palette(en_layer_palette),
    .en_sprite_palette(en_sprite_palette),
    .en_audio_filters(en_audio_filters),

    .sprite_freeze(dbg_sprite_freeze),

    .video_50hz(video_50hz),
    .video_57hz(video_57hz),
    .video_60hz(video_60hz)
);

// H/V offset
wire [3:0]	hoffset = status[20:17];
wire [3:0]	voffset = status[24:21];
jtframe_resync jtframe_resync
(
	.clk(CLK_VIDEO),
	.pxl_cen(ce_pix),
	.hs_in(hs_core),
	.vs_in(vs_core),
	.LVBL(~VBlank),
	.LHBL(~HBlank),
	.hoffset(hoffset),
	.voffset(voffset),
	.hs_out(HSync),
	.vs_out(VSync)
);

wire gamma_hsync, gamma_vsync, gamma_hblank, gamma_vblank;
wire [7:0] gamma_r, gamma_g, gamma_b;
gamma_fast video_gamma
(
    .clk_vid(CLK_VIDEO),
    .ce_pix(ce_pix),
    .gamma_bus(gamma_bus),
    .HSync(HSync),
    .VSync(VSync),
    .HBlank(HBlank),
    .VBlank(VBlank),
    .DE(),
    .RGB_in({R, G, B}),
    .HSync_out(gamma_hsync),
    .VSync_out(gamma_vsync),
    .HBlank_out(gamma_hblank),
    .VBlank_out(gamma_vblank),
    .DE_out(),
    .RGB_out({gamma_r, gamma_g, gamma_b})
);

wire VGA_DE_MIXER;
video_mixer #(386, 0, 0) video_mixer(
    .CLK_VIDEO(CLK_VIDEO),
    .CE_PIXEL(CE_PIXEL),
    .ce_pix(ce_pix),

    .scandoubler(forced_scandoubler || scandoubler_fx != 2'b00),
    .hq2x(0),

    .gamma_bus(),

    .R(gamma_r),
    .G(gamma_g),
    .B(gamma_b),

    .HBlank(gamma_hblank),
    .VBlank(gamma_vblank),
    .HSync(gamma_hsync),
    .VSync(gamma_vsync),

    .VGA_R(VGA_R),
    .VGA_G(VGA_G),
    .VGA_B(VGA_B),
    .VGA_VS(VGA_VS),
    .VGA_HS(VGA_HS),
    .VGA_DE(VGA_DE_MIXER),

    .HDMI_FREEZE(HDMI_FREEZE)
);


video_freak video_freak(
    .CLK_VIDEO(CLK_VIDEO),
    .CE_PIXEL(CE_PIXEL),
    .VGA_VS(VGA_VS),
    .HDMI_WIDTH(HDMI_WIDTH),
    .HDMI_HEIGHT(HDMI_HEIGHT),
    .VGA_DE(VGA_DE),
    .VIDEO_ARX(VIDEO_ARX),
    .VIDEO_ARY(VIDEO_ARY),

    .VGA_DE_IN(VGA_DE_MIXER),
    .ARX((!ar) ? ( no_rotate ? 12'd4 : 12'd3 ) : (ar - 1'd1)),
    .ARY((!ar) ? ( no_rotate ? 12'd3 : 12'd4 ) : 12'd0),
    .CROP_SIZE(crop_240p ? 240 : 0),
    .CROP_OFF(crop_offset),
    .SCALE(scale)
);


pause pause(
    .clk_sys(clk_sys),
    .reset(reset),
    .user_button(m_pause),
    .pause_request(0),
    .options({1'b0, pause_in_osd}),
    .pause_cpu(system_pause),
    .OSD_STATUS(OSD_STATUS)
);

`ifndef M72_DEBUG // debug uses DDR
screen_rotate screen_rotate(.*);
`endif



ddr_debug_data_t ddr_debug_data;

`ifdef M72_DEBUG
wire debug_stall;
ddr_debug ddr_debug(
    .*,
    .data(ddr_debug_data),
    .clk(CLK_96M),
    .reset(reset | ~pll_locked),
    .stall(debug_stall)
);
`endif

endmodule
