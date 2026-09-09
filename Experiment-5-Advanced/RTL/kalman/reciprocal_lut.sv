`timescale 1ns/1ps
module reciprocal_lut #(
    parameter integer ADDR_W = 6,
    parameter integer DATA_W = 29
)(
    input  logic [ADDR_W-1:0] addr,
    output logic signed [DATA_W-1:0] data
);
    always_comb begin
        case (addr)
            6'd0: data = 29'sd133177280;
            6'd1: data = 29'sd131144040;
            6'd2: data = 29'sd129171949;
            6'd3: data = 29'sd127258290;
            6'd4: data = 29'sd125400505;
            6'd5: data = 29'sd123596181;
            6'd6: data = 29'sd121843044;
            6'd7: data = 29'sd120138945;
            6'd8: data = 29'sd118481856;
            6'd9: data = 29'sd116869858;
            6'd10: data = 29'sd115301135;
            6'd11: data = 29'sd113773968;
            6'd12: data = 29'sd112286727;
            6'd13: data = 29'sd110837866;
            6'd14: data = 29'sd109425918;
            6'd15: data = 29'sd108049492;
            6'd16: data = 29'sd106707262;
            6'd17: data = 29'sd105397970;
            6'd18: data = 29'sd104120419;
            6'd19: data = 29'sd102873468;
            6'd20: data = 29'sd101656031;
            6'd21: data = 29'sd100467071;
            6'd22: data = 29'sd99305602;
            6'd23: data = 29'sd98170681;
            6'd24: data = 29'sd97061408;
            6'd25: data = 29'sd95976923;
            6'd26: data = 29'sd94916404;
            6'd27: data = 29'sd93879067;
            6'd28: data = 29'sd92864158;
            6'd29: data = 29'sd91870958;
            6'd30: data = 29'sd90898779;
            6'd31: data = 29'sd89946959;
            6'd32: data = 29'sd89014866;
            6'd33: data = 29'sd88101893;
            6'd34: data = 29'sd87207458;
            6'd35: data = 29'sd86331001;
            6'd36: data = 29'sd85471986;
            6'd37: data = 29'sd84629897;
            6'd38: data = 29'sd83804240;
            6'd39: data = 29'sd82994537;
            6'd40: data = 29'sd82200331;
            6'd41: data = 29'sd81421181;
            6'd42: data = 29'sd80656663;
            6'd43: data = 29'sd79906368;
            6'd44: data = 29'sd79169904;
            6'd45: data = 29'sd78446891;
            6'd46: data = 29'sd77736965;
            6'd47: data = 29'sd77039772;
            6'd48: data = 29'sd76354974;
            6'd49: data = 29'sd75682243;
            6'd50: data = 29'sd75021263;
            6'd51: data = 29'sd74371728;
            6'd52: data = 29'sd73733344;
            6'd53: data = 29'sd73105826;
            6'd54: data = 29'sd72488900;
            6'd55: data = 29'sd71882298;
            6'd56: data = 29'sd71285764;
            6'd57: data = 29'sd70699050;
            6'd58: data = 29'sd70121915;
            6'd59: data = 29'sd69554126;
            6'd60: data = 29'sd68995459;
            6'd61: data = 29'sd68445694;
            6'd62: data = 29'sd67904621;
            6'd63: data = 29'sd67372036;
            default: data = 29'sd0;
        endcase
    end
endmodule
