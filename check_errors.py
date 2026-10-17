import sys
from pymodbus.client import ModbusTcpClient

ERROR_FLAGS = {
    0x01: "Locked-Rotor (jam / mechanical stall)",
    0x02: "Over-Temperature (exceeds limit; auto-resets when cooled)",
    0x04: "Overcurrent (>2000 mA draw)",
    0x08: "Abnormal Motor Operation (drive stage / encoder mismatch)",
    0x10: "Communication Error (bus timeout / malformed packet)",
}

JOINTS = ["Little", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotate"]


def decode_errors(code: int) -> list[str]:
    return [desc for mask, desc in ERROR_FLAGS.items() if code & mask]


def check_hand(ip: str = "192.168.123.211", port: int = 6000):
    client = ModbusTcpClient(ip, port=port, timeout=1.5)
    if not client.connect():
        print(f"Failed to connect to {ip}:{port}")
        return

    try:
        err_res = client.read_holding_registers(address=1606, count=6, device_id=1)
        temp_res = client.read_holding_registers(address=1618, count=6, device_id=1)
        curr_res = client.read_holding_registers(address=1594, count=6, device_id=1)

        if err_res.isError():
            print(f"Error reading registers 1606–1611: {err_res}")
            return

        err_regs = err_res.registers
        temps = temp_res.registers if not temp_res.isError() else [None] * 6
        currs = curr_res.registers if not curr_res.isError() else [None] * 6

        print(f"{'Joint':<14} {'Raw':<6} {'Temp':<8} {'Current':<10} {'Status / Errors'}")
        print("-" * 65)

        for i, joint in enumerate(JOINTS):
            code = err_regs[i]
            active = decode_errors(code)
            temp_str = f"{temps[i]}°C" if temps[i] is not None else "N/A"
            curr_str = f"{currs[i]} mA" if currs[i] is not None else "N/A"
            err_str = "OK" if not active else "; ".join(active)
            print(f"{joint:<14} 0x{code:02X}   {temp_str:<8} {curr_str:<10} {err_str}")

    finally:
        client.close()


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else "192.168.123.211"
    # Offline decode if given a numeric or hex code (e.g., 0x05 or 3)
    try:
        raw_code = int(arg, 0)
        errors = decode_errors(raw_code)
        print(f"Code 0x{raw_code:02X} ({raw_code}):")
        print("\n".join(f" - {e}" for e in errors) if errors else " - OK (no errors)")
    except ValueError:
        check_hand(ip=arg)
