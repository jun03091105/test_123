module debouncer(
    input clk,
    input I,          // 버튼 입력
    output reg O      // 디바운싱된 출력
    );
    
    reg [19:0] cnt = 0;  // 카운터 크기를 늘려 디바운싱 시간 조절
    reg Iv = 0;
    
    always@(posedge clk)
    begin
        if (I == Iv)
        begin
            if (cnt == 20'd999_999)  // 약 10ms (100MHz 클록 기준)
                O <= I;
            else
                cnt <= cnt + 1;
        end
        else
        begin
            cnt <= 0;
            Iv <= I;
        end
    end
    
endmodule
