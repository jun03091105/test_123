`timescale 1ns / 1ps

// Debouncer module
module debouncer(
    input clk,
    input I,
    output reg O
    );
    
    reg [19:0] cnt = 0;  
    reg Iv = 0;
    
    always @(posedge clk) begin
        if (I == Iv) begin
            if (cnt == 20'd999_999)  
                O <= I;
            else
                cnt <= cnt + 1;
        end else begin
            cnt <= 0;
            Iv <= I;
        end
    end
    
endmodule

module SS_Decoder(
    input clock,
    input [3:0] A,       // Floor switch input
    input BTNC,          // N17 button (start moving)
    input BTNU,          // M18 button (increase alarm counter)
    input BTND,          // P18 button (decrease alarm counter)
    input BTNL,          // P17 button (set error flag - emergency stop)
    input BTNR,          // M17 button (clear error flag)
    output CA, CB, CC, CD, CE, CF, CG, CDP,
    output AN7, AN6, AN5, AN4, AN3, AN2, AN1, AN0,
    output AUD_PWM,
    output SUM0, SUM1, SUM2, SUM3
    );

    // Debounced signals
    wire BTNC_db, BTNU_db, BTND_db, BTNL_db, BTNR_db;

    debouncer db_c(.clk(clock), .I(BTNC), .O(BTNC_db));
    debouncer db_u(.clk(clock), .I(BTNU), .O(BTNU_db));
    debouncer db_d(.clk(clock), .I(BTND), .O(BTND_db));
    debouncer db_l(.clk(clock), .I(BTNL), .O(BTNL_db));
    debouncer db_r(.clk(clock), .I(BTNR), .O(BTNR_db));

    reg [7:0] cathodedata;
    reg [7:0] anodedata;
    reg [2:0] digit = 0;
    reg [3:0] data;
    reg setdp = 0;
    reg [31:0] counter = 0;
    reg [3:0] current_floor = 4'd1;
    reg [3:0] goal_floor = 4'd1;
    reg [31:0] move_counter = 0;
    reg [31:0] anim_counter = 0;
    reg anim_state = 0;
    reg [31:0] door_timer = 0;
    reg [31:0] audio_timer = 0;

    reg [31:0] pwm_counter = 0;
    reg pwm_out = 0;

    localparam integer PWM_FREQ = 1000;
    localparam integer PWM_PERIOD = 100_000_000 / (2 * PWM_FREQ);

    localparam STATE_IDLE = 3'd0;
    localparam STATE_DOOR_CLOSING = 3'd1;
    localparam STATE_MOVING = 3'd2;
    localparam STATE_DOOR_OPENING = 3'd3;
    localparam STATE_PLAY_SOUND = 3'd4;
    localparam STATE_ERROR = 3'd5;
    reg [2:0] elevator_state = STATE_IDLE;
    reg [2:0] prev_elevator_state = STATE_IDLE;

    reg BTNC_db_prev = 0, BTNU_db_prev = 0, BTND_db_prev = 0, BTNL_db_prev = 0, BTNR_db_prev = 0;

    localparam DOOR_CLOSED_TOP = 8'b00001111;
    localparam DOOR_CLOSED_BOTTOM = 8'b01100011;
    localparam DOOR_OPEN_TOP = 8'b01100011;
    localparam DOOR_OPEN_BOTTOM = 8'b00001111;

    reg [3:0] floor_request = 4'b0000;

    reg [3:0] alarm_counter = 4'd0;
    reg err_flag = 0;
    reg err_flag_by_button = 0;

    assign CA = cathodedata [7];
    assign CB = cathodedata [6];
    assign CC = cathodedata [5];
    assign CD = cathodedata [4];
    assign CE = cathodedata [3];
    assign CF = cathodedata [2];
    assign CG = cathodedata [1];
    assign CDP = cathodedata [0];
    assign AN7 = anodedata [7];
    assign AN6 = anodedata [6];
    assign AN5 = anodedata [5];
    assign AN4 = anodedata [4];
    assign AN3 = anodedata [3];
    assign AN2 = anodedata [2];
    assign AN1 = anodedata [1];
    assign AN0 = anodedata [0];

    assign AUD_PWM = pwm_out;
    assign SUM0 = floor_request[0]; // 1F LED
    assign SUM1 = floor_request[1]; // 2F LED
    assign SUM2 = floor_request[2]; // 3F LED
    assign SUM3 = floor_request[3]; // 4F LED

    // Finds closest floor with a request
    function [3:0] find_closest_floor;
        input [3:0] curr;
        input [3:0] requests;
        integer i;
        reg [3:0] closest;
        reg [3:0] dist_min;
        reg [3:0] floor_idx;
        reg [3:0] dist;
        begin
            closest = 4'd0;
            dist_min = 4'd15;
            for(i=0; i<4; i=i+1) begin
                if (requests[i] == 1) begin
                    floor_idx = i+1;
                    dist = (curr > floor_idx) ? (curr - floor_idx) : (floor_idx - curr);
                    if (dist < dist_min) begin
                        dist_min = dist;
                        closest = floor_idx;
                    end
                end
            end
            find_closest_floor = closest;
        end
    endfunction

    always @(posedge clock) begin
        // Edge detect
        BTNC_db_prev <= BTNC_db;
        BTNU_db_prev <= BTNU_db;
        BTND_db_prev <= BTND_db;
        BTNL_db_prev <= BTNL_db;
        BTNR_db_prev <= BTNR_db;

        // Alarm counter
        if (BTNU_db && !BTNU_db_prev && alarm_counter < 4'd15)
            alarm_counter <= alarm_counter + 1;
        if (BTND_db && !BTND_db_prev && alarm_counter > 4'd0)
            alarm_counter <= alarm_counter - 1;

        // Error flag set/clear
        if (BTNL_db && !BTNL_db_prev)
            err_flag_by_button <= 1;
        else if (BTNR_db && !BTNR_db_prev)
            err_flag_by_button <= 0;

        // Determine err_flag
        if (err_flag_by_button || (alarm_counter > 4'd3))
            err_flag <= 1;
        else
            err_flag <= 0;

        // Immediately go to error if err_flag is set and not already in error
        if (err_flag && (elevator_state != STATE_ERROR)) begin
            prev_elevator_state <= elevator_state;
            elevator_state <= STATE_ERROR;
        end else if (!err_flag && elevator_state == STATE_ERROR) begin
            // Return to prev state if error cleared
            elevator_state <= prev_elevator_state;
        end

        // State machine
        case (elevator_state)
            STATE_IDLE: if (!err_flag) begin
                door_timer <= 0;
                audio_timer <= 0;

                // Floor request update (exclude current floor)
                if (A[0] && current_floor != 4'd1) floor_request[0] <= 1'b1;
                if (A[1] && current_floor != 4'd2) floor_request[1] <= 1'b1;
                if (A[2] && current_floor != 4'd3) floor_request[2] <= 1'b1;
                if (A[3] && current_floor != 4'd4) floor_request[3] <= 1'b1;

                // If requests exist, find closest
                if (floor_request != 4'b0000)
                    goal_floor <= find_closest_floor(current_floor, floor_request);
                else
                    goal_floor <= current_floor;

                // Start moving if N17 pressed once and we have a valid goal
                if (BTNC_db && !BTNC_db_prev && (floor_request != 4'b0000) && (goal_floor != current_floor))
                    elevator_state <= STATE_DOOR_CLOSING;
            end
            STATE_DOOR_CLOSING: if (!err_flag) begin
                door_timer <= door_timer + 1;
                if (door_timer >= 50_000_000) begin
                    door_timer <= 0;
                    elevator_state <= STATE_MOVING;
                end
            end
            STATE_MOVING: if (!err_flag) begin
                if (current_floor != goal_floor) begin
                    move_counter <= move_counter + 1;
                    anim_counter <= anim_counter + 1;
                    if (move_counter >= 200_000_000) begin
                        move_counter <= 0;
                        if (current_floor < goal_floor)
                            current_floor <= current_floor + 1;
                        else
                            current_floor <= current_floor - 1;

                        // If request at this floor, stop immediately
                        if (floor_request[current_floor-1] == 1) begin
                            floor_request[current_floor-1] <= 0;
                            elevator_state <= STATE_PLAY_SOUND;
                        end else if (current_floor == goal_floor) begin
                            if (floor_request[current_floor-1] == 1)
                                floor_request[current_floor-1] <= 0;
                            elevator_state <= STATE_PLAY_SOUND;
                        end
                    end

                    if (anim_counter >= 20_000_000) begin
                        anim_counter <= 0;
                        anim_state <= ~anim_state;
                    end
                end else begin
                    if (floor_request[current_floor-1] == 1)
                        floor_request[current_floor-1] <= 0;

                    move_counter <= 0;
                    anim_counter <= 0;
                    anim_state <= 0;
                    elevator_state <= STATE_PLAY_SOUND;
                end
            end
            STATE_PLAY_SOUND: if (!err_flag) begin
                audio_timer <= audio_timer + 1;
                if (audio_timer >= 100_000_000) begin
                    audio_timer <= 0;
                    elevator_state <= STATE_DOOR_OPENING;
                end
            end
            STATE_DOOR_OPENING: if (!err_flag) begin
                door_timer <= door_timer + 1;
                if (door_timer >= 50_000_000) begin
                    door_timer <= 0;
                    elevator_state <= STATE_IDLE;
                end
            end
            STATE_ERROR: begin
                // Stay here until err_flag is cleared, then return to prev state
            end
            default: elevator_state <= STATE_IDLE;
        endcase

        // PWM for sound
        if ((elevator_state == STATE_PLAY_SOUND) || err_flag) begin
            pwm_counter <= pwm_counter + 1;
            if (pwm_counter >= PWM_PERIOD) begin
                pwm_counter <= 0;
                pwm_out <= ~pwm_out; 
            end
        end else begin
            pwm_out <= 1'b0;
            pwm_counter <= 0;
        end

        // 7-seg scan
        counter <= counter + 1;
        if (counter >= 100000) begin
            counter <= 0;
            digit <= digit + 1;
            if (digit >= 8)
                digit <= 0;
        end
    end

    always @ (*) begin
        anodedata = 8'b11111111;
        cathodedata = 8'b11111111;

        case (digit)
            7:
                if (err_flag) begin
                    anodedata[digit] = 0;
                    cathodedata = 8'b01100001; // 'E'
                end
            6:
                if (err_flag) begin
                    anodedata[digit] = 0;
                    cathodedata = 8'b01110011; // 'r'
                end
            5:
                if (err_flag) begin
                    anodedata[digit] = 0;
                    cathodedata = 8'b01110011; // 'r'
                end
            4: begin
                anodedata[digit] = 0;
                data = alarm_counter;
                case (data)
                    4'd0: cathodedata = 8'b00000011; //0
                    4'd1: cathodedata = 8'b10011111; //1
                    4'd2: cathodedata = 8'b00100101; //2
                    4'd3: cathodedata = 8'b00001101; //3
                    4'd4: cathodedata = 8'b10011001; //4
                    4'd5: cathodedata = 8'b01001001; //5
                    4'd6: cathodedata = 8'b01000001; //6
                    4'd7: cathodedata = 8'b00011111; //7
                    4'd8: cathodedata = 8'b00000001; //8
                    4'd9: cathodedata = 8'b00001001; //9
                    default: cathodedata = 8'b11111111;
                endcase
            end
            3: begin
                anodedata[digit] = 0;
                if ((elevator_state == STATE_IDLE || elevator_state == STATE_DOOR_OPENING) ||
                    (elevator_state == STATE_ERROR && (prev_elevator_state == STATE_IDLE || prev_elevator_state == STATE_DOOR_OPENING)))
                    cathodedata = DOOR_OPEN_TOP;
                else
                    cathodedata = DOOR_CLOSED_TOP;
            end
            2: begin
                anodedata[digit] = 0;
                if ((elevator_state == STATE_IDLE || elevator_state == STATE_DOOR_OPENING) ||
                    (elevator_state == STATE_ERROR && (prev_elevator_state == STATE_IDLE || prev_elevator_state == STATE_DOOR_OPENING)))
                    cathodedata = DOOR_OPEN_BOTTOM;
                else
                    cathodedata = DOOR_CLOSED_BOTTOM;
            end
            1: begin
                anodedata[digit] = 0;
                if (elevator_state == STATE_MOVING) begin
                    if (current_floor < goal_floor)
                        data = anim_state ? 4'd11 : 4'd10; // FAB or EGC
                    else
                        data = anim_state ? 4'd13 : 4'd12; // EDC or FGB

                    case (data)
                        4'd10: cathodedata = 8'b11010101; // EGC
                        4'd11: cathodedata = 8'b00111011; // FAB
                        4'd12: cathodedata = 8'b10111001; // FGB
                        4'd13: cathodedata = 8'b11000111; // EDC
                        default: cathodedata = 8'b11111111;
                    endcase
                end else begin
                    cathodedata = 8'b11111111;
                end
            end
            0: begin
                anodedata[digit] = 0;
                data = current_floor;
                case (data)
                    4'd0: cathodedata = 8'b00000011; 
                    4'd1: cathodedata = 8'b10011111; 
                    4'd2: cathodedata = 8'b00100101; 
                    4'd3: cathodedata = 8'b00001101; 
                    4'd4: cathodedata = 8'b10011001; 
                    default: cathodedata = 8'b11111111;
                endcase
            end
        endcase
    end

endmodule
