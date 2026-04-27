#!/usr/bin/python3
import argparse
import urllib.parse
import numpy as np


def rgba_conversion(arg: int) -> float:
    return round(float(arg) / 255, 2)

def rgba_service(arg_list: list) -> None:
    for i in arg_list:
        if (float(i) > 255 or float(i) < 0):
            print("[WARN] 参数范围应为 0~255")
            return
    if len(arg_list) < 3:
        print("[WARN] 参数不得少于三个")
    elif len(arg_list) == 3:
        print(f'rgba="{rgba_conversion(arg_list[0])} {rgba_conversion(arg_list[1])} {rgba_conversion(arg_list[2])} 1"')
    else:
        print(f'rgba"{rgba_conversion(arg_list[0])} {rgba_conversion(arg_list[1])} {rgba_conversion(arg_list[2])} {arg_list[3]}"')

def degree_service(radian: float) -> None:
    print(f"{np.degrees(radian):.2f}")

def radian_service(degrees: float) -> None:
    print(f"{np.radians(degrees):.4f}")

def url_encode_service(url: str):
    url_list = url.split("-")
    target_1 = urllib.parse.quote(url_list[0], safe='')
    target_2 = urllib.parse.quote(url_list[1], safe='')
    
    print(f"{target_1+'/'+target_2}")

def utl_decode_service(url: str):
    url_list = url.split("/")
    target_1 = urllib.parse.unquote(url_list[0])
    target_2 = urllib.parse.unquote(url_list[1])
    print(f"{target_1} {target_2}")


def main():
    # 初始化
    parser = argparse.ArgumentParser(description="将0~255范围的rgba转为0~1范围的rgba")
    
    # 添加
    parser.add_argument("--rgba", nargs='+', help="输入rgba，至少三个参数")
    parser.add_argument("-d", "--degree", type=float, help="弧度转角度")
    parser.add_argument("-r", "--radian", type=float, help="角度转弧度")
    parser.add_argument("-en", "--encode", help="URL编码，使用‘-’分隔话题和消息类型")
    parser.add_argument("-de", "--decode", help="URL反编码")

    # 解析
    args = parser.parse_args()

    # rgba参数
    if args.rgba:
        rgba_service(args.rgba)
    if args.degree:
        degree_service(args.degree)
    if args.radian:
        radian_service(args.radian)
    if args.encode:
        url_encode_service(args.encode)
    if args.decode:
        utl_decode_service(args.decode)


if __name__ == "__main__":
    main()
