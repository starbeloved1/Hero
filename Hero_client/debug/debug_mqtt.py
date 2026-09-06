#!/usr/bin/env python3
"""
快速调试脚本：检查MQTT收到的300B数据内容
"""

import paho.mqtt.client as mqtt
import struct
import time

CONFIG = {
    'broker': '192.168.12.1',  
    'mqtt_port': 3333,         
    'client_id': '101',        # 蓝方英雄=101，红方英雄=1
    'topic': 'CustomByteBlock',
    'qos': 1,
}

FRAME_SIZE = 303
DATA_SIZE = 300
packet_count = 0

def bytes_to_hex(data, max_len=40):
    return ' '.join(f'{b:02X}' for b in data[:max_len])

def on_connect(client, userdata, flags, rc, props):
    print(f'[MQTT] 已连接，订阅话题: {CONFIG["topic"]}')
    client.subscribe(CONFIG['topic'], qos=CONFIG['qos'])

def on_message(client, userdata, msg):
    global packet_count
    payload = msg.payload

    packet_count += 1
    print(f'\n=== 包 #{packet_count} (len={len(payload)}) ===')

    # 检查长度
    if len(payload) != FRAME_SIZE:
        print(f'  [警告] 长度不是303字节!')
        print(f'  原始数据前40字节: {bytes_to_hex(payload)}')
        return

    # 检查protobuf tag
    if payload[0] != 0x0A:
        print(f'  [警告] protobuf tag错误: 0x{payload[0]:02X} (期望0x0A)')
        print(f'  原始数据前40字节: {bytes_to_hex(payload)}')
        return

    # 解析varint长度
    idx = 1
    length = 0
    shift = 0
    while True:
        byte = payload[idx]
        length |= (byte & 0x7F) << shift
        idx += 1
        if byte < 0x80:
            break
        shift += 7

    print(f'  protobuf varint长度: {length} (期望300)')
    data_start = idx

    # 提取300B数据
    data_section = payload[data_start:data_start + DATA_SIZE]

    # 解析header
    sequence_id = struct.unpack('<Q', data_section[0:8])[0]
    h264_chunk = data_section[8:DATA_SIZE]

    print(f'  sequence_id: {sequence_id}')
    print(f'  H264 chunk长度: {len(h264_chunk)} 字节')
    print(f'  H264 chunk前20字节: {bytes_to_hex(h264_chunk, 20)}')

    # 检查NAL起始码
    has_nal_3byte = h264_chunk[:3] == b'\x00\x00\x01'
    has_nal_4byte = h264_chunk[:4] == b'\x00\x00\x00\x01'
    print(f'  NAL起始码(00 00 01): {has_nal_3byte}')
    print(f'  NAL起始码(00 00 00 01): {has_nal_4byte}')

    # 搜索NAL起始码
    nal_positions = []
    for i in range(len(h264_chunk) - 2):
        if h264_chunk[i:i+3] == b'\x00\x00\x01':
            nal_positions.append(i)
        elif i < len(h264_chunk) - 3 and h264_chunk[i:i+4] == b'\x00\x00\x00\x01':
            nal_positions.append(i)

    if nal_positions:
        print(f'  发现NAL起始码位置: {nal_positions[:5]}')
        # 打印NAL类型
        for pos in nal_positions[:3]:
            # 00 00 01后的字节是NAL header
            nal_type_byte = h264_chunk[pos+3] if pos+3 < len(h264_chunk) else 0
            nal_type = nal_type_byte & 0x1F
            nal_type_names = {
                1: 'Non-IDR Slice',
                5: 'IDR Slice',
                6: 'SEI',
                7: 'SPS',
                8: 'PPS',
                9: 'AUD',
            }
            nal_name = nal_type_names.get(nal_type, f'Unknown({nal_type})')
            print(f'    pos={pos}: NAL type={nal_type} ({nal_name})')
    else:
        print(f'  [警告] 没有发现NAL起始码!')

    if packet_count >= 20:
        print('\n[调试完成] 已收到20个包，退出')
        client.disconnect()

def main():
    print('=== MQTT调试脚本启动 ===')
    print(f'配置: broker={CONFIG["broker"]}, port={CONFIG["mqtt_port"]}, topic={CONFIG["topic"]}')
    print(f'client_id={CONFIG["client_id"]}')

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=CONFIG['client_id']
    )
    client.on_connect = on_connect
    client.on_message = on_message

    print(f'[连接] 正在连接 {CONFIG["broker"]}:{CONFIG["mqtt_port"]}...')
    client.connect(CONFIG['broker'], CONFIG['mqtt_port'], keepalive=60)
    print('[连接] 连接成功，开始循环...')
    client.loop_forever()

if __name__ == '__main__':
    main()
