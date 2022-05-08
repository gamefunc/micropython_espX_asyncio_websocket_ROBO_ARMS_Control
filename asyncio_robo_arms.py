"""
课程系列: micropython进阶拓展(esp32单片机开发)
https://edu.csdn.net/course/detail/29666
制作人信息:
    微信: gamefunc / 18576539615
    qq: 32686647
转载与修改需要附带以上信息;
"""

# 本节内容: websocket控制机械臂


"""
机械臂:
    s16_p = 爪上
    s16_m = 爪下

    s17.plus = 爪前
    s17.minus = 爪后

    s18.plus = 左转
    s18.minus = 右转

    s19.plus = 爪开
    s19.minus = 爪合
"""



import  hashlib, struct
import uasyncio as asyncio

import machine, ubinascii


class Servo_Control:
    def __init__(
            self,
            pin_num = 16,
            limit_min = 35,
            limit_max = 180,
            default_angle = 90,
            pwm_freq = 50,
            duty_range = 1023) -> None:
        # 保存一下传入参数:
        self.__pin_num = pin_num
        self.__pwm_freq = pwm_freq
        self.__duty_range = duty_range
        self.__limit_min = limit_min
        self.__limit_max = limit_max

        self.__now_angle = default_angle

        # 实例化pin, 并且弄成 pwm:
        self.__servo_pin = machine.Pin(pin_num, machine.Pin.OUT)
        self.__servo_pwm = machine.PWM(
            self.__servo_pin, 
            freq = self.__pwm_freq,
            duty = 0
        )

        # 开头先转到要求的位置;
        self.turn(self.__now_angle)



    def __calc_angle_duty(self, angle: float) -> int:
        """
        计算需要转到的角度, 需要设置  多少 duty;
        """
        if (angle > self.__limit_max) or (angle < self.__limit_min):
            raise ValueError(f"转角只能在{self.__limit_max} -> {self.__limit_min}度之间")

        duty_ratio = ((2 / 180) * angle + 0.5) / (1000 / self.__pwm_freq) * (self.__duty_range)
        # print(duty_ratio)
        self.__now_angle = angle
        print(self.__now_angle)

        return int(duty_ratio)


    def plus(self, n = 1) -> None:
        """
        转到 + n 度
        """
        self.turn(self.__now_angle + n)


    def minus(self, n = 1) -> None:
        """
        转到 - n 度
        """
        self.turn(self.__now_angle - n)


    def turn(self, angle: float) -> None:
        """
        让 servo 电机转:
        """
        self.__servo_pwm.duty(
            self.__calc_angle_duty(angle))

    def get_this_servo_status_string(self) -> str:
        """
        获取该电机当前状态:
        "16[35 -> 180]: 99"
        """
        s = f"{self.__pin_num}[{self.__limit_min} -> {self.__limit_max}]: "
        s += f"{self.__now_angle}"

        return s




ROBO_ARMS = {
    16 : Servo_Control(16, 90, 180, 180),
    17 : Servo_Control(17, 35, 180, 90),
    18 : Servo_Control(18, 0, 180, 90),
    19 : Servo_Control(19, 13, 20, 16)
}







class WEB_CONTROL:
    def __init__(self) -> None:
        self.http_ctl = self.HTTP_CONTROL()
        self.ws_ctl = self.Websocket_Control()


    class HTTP_CONTROL:
        def parse_request_header(self, req: bytes) -> dict:
            """
            简单解析 request header, 无视 url编码
            urllib.parse.unquote_plus()
            """
            # req_method, req_path, http_ver, kv_s = req.split(maxsplit=3)
            a, kv_s = req.split(b"\r\n", 1)
            req_method, req_path, http_ver = a.split(b" ", 3)



            result = {
                b"req_method": req_method,
                b"req_path": req_path,
                b"http_ver": http_ver
            }

            for kv in kv_s.split(b"\r\n"):
                if len(kv.strip()) < 3: continue
                k, v = kv.split(b": ", 1)
                result[k] = v

            return result


        def warp_http_1_1_rsp_common(
                self, html_data: bytes) -> bytes:
            """
            包装 http 1.1 keep alive;
            """
            rsp = b"HTTP/1.1 200 OK \r\n"
            rsp += b"Server: Gamefunc\r\n"
            data_len = str(len(html_data)).encode("utf-8")
            rsp = rsp + b"Content-Length: " + data_len + b"\r\n"
            rsp += b"Connection: keep-alive\r\n"
            rsp += b"\r\n"

            return rsp + html_data


    class Websocket_Control:

        def __init__(self) -> None:
            opcode_kv = {
                "cont": 0x00,
                "text": 0x01,
                "binary": 0x02,
                "close": 0x08,
                "ping": 0x09,
                "pong": 0x0a 
            }
            opcode_vk = dict(map(reversed, opcode_kv.items()))
            
        def __get_websocket_accept_key(
                self, websocket_req_key: bytes) -> bytes:
            """
            把客服端的websocket req key 转为 accept key;
            传入 "Sec-WebSocket-Key"
            """
            ws_GUILD = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
            ws_accept_key = websocket_req_key + ws_GUILD

            # return base64.b64encode(hashlib.sha1(ws_accept_key).digest())
            return ubinascii.b2a_base64(hashlib.sha1(ws_accept_key).digest())[0: -1]

        def create_response_header(
                self, req_header_dict: dict) -> bytes:
            """
            通过 request header 创建
            websocket 的 response header:
            HTTP/1.1 101 Switching Protocols
            Upgrade: websocket
            Connection: Upgrade
            Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
            Sec-WebSocket-Protocol: chat
            """
            ws_accpet_key = self.__get_websocket_accept_key(
                req_header_dict[b'Sec-WebSocket-Key']
            )
            ws_rsp_header = b"HTTP/1.1 101 Switching Protocols\r\n"
            ws_rsp_header += b"Upgrade: websocket\r\n"
            ws_rsp_header += b"Connection: Upgrade\r\n"
            ws_rsp_header += b"Sec-WebSocket-Accept: "
            ws_rsp_header += ws_accpet_key
            ws_rsp_header += b"\r\n"
            ws_rsp_header += b"Server: gamefunc\r\n"
            ws_rsp_header += b"\r\n"
            return ws_rsp_header


        def __mask_trans(self, payload: bytes, mask: bytes) -> bytes:
            """
            mask 转换; 
            """
            result = bytearray(len(payload))

            for i in range(len(payload)):
                result[i] = payload[i] ^ mask[i % 4]

            return result


        def parse_frame(self, data: bytes) -> dict:
            """
            解析cli发送过来的ws frame, 返回字典:
                dict = {
                    frame_len: 如果是payload长度不足的话, 就是0, 否则就是完整:
                        frame的长度不足, 这里是0,
                        如果frame长度够, 就是payload长度足够, 返回frame len,
                        让外面 full_data[frame_len: ]

                    opcode: 

                    payload: bytes;
                }
            """

            head1, head2 = struct.unpack("!BB", data[0: 2])
            # frame 第1个字节;
            _opcode = head1 & 0b00001111
            if _opcode not in (1,2,9,10): raise

            # frame 第2个字节;
            is_mask = head2 & 0b10000000
            payload_len = head2 & 0b01111111
            if payload_len >= 126: raise 

            payload_start_pos = 6 if is_mask else 2

            # 如果没收完整:
            if len(data) < (payload_start_pos + payload_len):
                return {"frame_len": 0}

            # frame 完整:
            payload = data[payload_start_pos: payload_start_pos + payload_len]
            if is_mask: 
                mask = data[2:6]
                payload = self.__mask_trans(payload, mask)

            return {
                "frame_len": payload_start_pos + payload_len,
                "opcode": _opcode,
                "payload": payload
            }
        
        def warp_payload_to_frame(
                self, payload: bytes,
                opcode_num = 1) -> bytes:
            """
            把 payload 包装为frame:
            """
            head1 = struct.pack("!B", 0b10000000 | opcode_num)
            if len(payload) < 126:
                head2 = struct.pack("!B", 0 | len(payload))
            elif len(payload) <= 65535:
                head2 = struct.pack("!B", 0 | 126)
                head2 += struct.pack("!H", len(payload))
            return head1 + head2 + payload



            

            

WEB_CTL = WEB_CONTROL()

async def cron_send_data(writer: asyncio.StreamWriter) -> None:
    i = 0
    while 1:
        await asyncio.sleep(3)
        writer.write(
           WEB_CTL.ws_ctl.warp_payload_to_frame( 
               f"ding_qi_shuju_{i}".encode("utf-8"))
        )
        i += 1
        await writer.drain()

async def handle_echo(reader, writer):
    global WEB_CTL
    global ROBO_ARMS

    MAIN_PAGE_HTML_CODE = b""
    with open("robo_arm.html", "rb") as f:
        MAIN_PAGE_HTML_CODE = f.read()


    full_data = b""
    while 1:
        data = await reader.read(4096)
        if not data: break
        full_data += data
        if b"\r\n\r\n" not in full_data:
            continue

        req_header_dict = WEB_CTL.http_ctl.parse_request_header(data)
        path = req_header_dict[b"req_path"]
        print(f"{path}")

        if path == b"/servo_control":
            ws_accept_header = WEB_CTL.ws_ctl.create_response_header(
                req_header_dict)
            await writer.awrite(ws_accept_header)

            # cron_task = asyncio.create_task(cron_send_data(writer))
            full_data = b""
            while 1:
                data = await reader.read(4096)
                if not data: raise 
                # 有可能full data 里面有 好几个 frame:
                full_data += data
                while 1:
                    if len(full_data) < 2: break
                    payload_dict = WEB_CTL.ws_ctl.parse_frame(full_data)
                    if payload_dict["frame_len"] == 0: break
                    full_data = full_data[payload_dict["frame_len"]: ]


                    payload = payload_dict["payload"]
                    print(payload)

                    """
                    协议: 16_0.5_p: 
                        ROBO_ARMS的servo_num
                        每次plus/minus 多少度
                        p = plus, m = minus
                    """
                    # 分离协议:
                    units = payload.decode("utf-8").split("_")
                    if len(units) != 3: continue
                    print(units)

                    servo_num = int(units[0])
                    num_of_step = float(units[1])

                    try:
                        if units[2] == "p":
                            ROBO_ARMS[servo_num].plus(num_of_step)
                        else:
                            ROBO_ARMS[servo_num].minus(num_of_step)
                    except:
                        pass

                    # 给 客服端 返回 信息:
                    status = ""
                    for k in ROBO_ARMS:
                        status += ROBO_ARMS[k].get_this_servo_status_string()
                        status += "<br>"

                    await writer.awrite(
                        WEB_CTL.ws_ctl.warp_payload_to_frame(
                            status.encode("utf-8"))
                    )

        else: # "/", "other"
            writer.write(
                WEB_CTL.http_ctl.warp_http_1_1_rsp_common(
                    MAIN_PAGE_HTML_CODE))

        full_data = b""
        await writer.drain()

    # await writer.aclose()
    writer.close()




async def main():
    loop = asyncio.get_event_loop()
    server = asyncio.start_server(
        handle_echo, '0.0.0.0', 80)
    asyncio.create_task(server)

    loop.run_forever()
    # while 1:
    #     await asyncio.sleep(100)

asyncio.run(main())


