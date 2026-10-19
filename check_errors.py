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
    # Check both high byte and low byte
    byte_val = (code >> 8) if (code >> 8) != 0 else (code & 0xFF)
    return [desc for mask, desc in ERROR_FLAGS.items() if byte_val & mask]


def check_hand(ip: str = "192.168.123.210", port: int = 6000, clear: bool = False):
    client = ModbusTcpClient(ip, port=port, timeout=1.5)
    if not client.connect():
        print(f"Failed to connect to {ip}:{port}")
        return

    try:
        if clear:
            print("[check_errors] Sending CLEAR_ERROR (register 1004)...")
            client.write_registers(address=1004, values=[1], device_id=1)

        err_res = client.read_holding_registers(address=1606, count=6, device_id=1)
        temp_res = client.read_holding_registers(address=1618, count=6, device_id=1)
        curr_res = client.read_holding_registers(address=1594, count=6, device_id=1)

        if err_res.isError():
            print(f"Error reading registers 1606–1611: {err_res}")
            return

        err_regs = err_res.registers
        raw_temps = temp_res.registers if not temp_res.isError() else [None] * 6
        currs = curr_res.registers if not curr_res.isError() else [None] * 6

        print(f"{'Joint':<14} {'Raw':<8} {'Temp':<8} {'Current':<10} {'Status / Errors'}")
        print("-" * 65)

        for i, joint in enumerate(JOINTS):
            code = err_regs[i]
            active = decode_errors(code)
            raw_t = raw_temps[i]
            t_val = (raw_t & 0xFF) if raw_t is not None else None
            temp_str = f"{t_val}°C" if t_val is not None else "N/A"
            curr_str = f"{currs[i]} mA" if currs[i] is not None else "N/A"
            err_str = "OK" if not active else "; ".join(active)
            print(f"{joint:<14} 0x{code:04X}   {temp_str:<8} {curr_str:<10} {err_str}")

    finally:
        client.close()


if __name__ == "__main__":
    clear_flag = "--clear" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--clear"]
    arg = args[0] if len(args) > 0 else "192.168.123.210"

    try:
        raw_code = int(arg, 0)
        errors = decode_errors(raw_code)
        print(f"Code 0x{raw_code:04X} ({raw_code}):")
        print("\n".join(f" - {e}" for e in errors) if errors else " - OK (no errors)")
    except ValueError:
        check_hand(ip=arg, clear=clear_flag)
