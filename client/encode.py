import ast
import json
import re
from typing import Any

_FUNC_RE = re.compile(r"""
    ^\s*
    (?P<name>[A-Za-z_]\w*)     # function name
    \s*\(
      (?P<args>.*)             # argument list (may be empty)
    \)\s*$
""", re.VERBOSE | re.DOTALL)

def _parse_args(arg_src: str) -> list[Any]:
    """
    Parse the comma-separated argument list using Python's literal syntax.
    Accepts ints, floats, and quoted strings (single or double quotes).
    Examples of valid arg lists:
      ""           -> []
      "10"         -> [10]
      "'abc', 2.5" -> ["abc", 2.5]
      " 'a, b', 3" -> ["a, b", 3]
    """
    arg_src = arg_src.strip()
    if not arg_src:
        return []
    # Wrap in brackets so ast.literal_eval can parse a Python list literal.
    try:
        parsed = ast.literal_eval(f"[{arg_src}]")
    except Exception as e:
        raise ValueError(f"Could not parse arguments: {e}") from e

    # Validate types (only str, int, float allowed)
    ok = (str, int, float)
    for i, v in enumerate(parsed):
        if not isinstance(v, ok):
            raise TypeError(f"Argument {i} has unsupported type {type(v).__name__}; "
                            f"only str, int, and float are allowed.")
    return list(parsed)

def encode_function_call(call: str) -> bytes:
    """
    Convert 'Name(args...)' into a null-terminated JSON string like:
      CalibratePull(10)   -> '{"function":{"CalibratePull":10}}\\0'
      Unlock()            -> '{"function":"Unlock"}\\0'
      Foo("a", 3, 2.5)    -> '{"function":{"Foo":["a",3,2.5]}}\\0'

    Returns a **Python str** ending with a NUL ('\\0').
    """
    m = _FUNC_RE.match(call)
    if not m:
        raise ValueError("Input must look like FunctionName(arg, ...)")
    name = m.group("name")
    args_src = m.group("args")
    args = _parse_args(args_src)

    if len(args) == 0:
        payload = {"function": name}                       # unit variant -> "Name"
    elif len(args) == 1:
        payload = {"function": {name: args[0]}}           # single-field tuple variant
    else:
        payload = {"function": {name: args}}              # multi-field tuple -> list

    json_text = json.dumps(payload, separators=(",", ":"))
    return (json_text).encode("utf-8")


def encode_message(call: str) -> bytes:


    return bytes(encode_function_call(call))

if __name__ == "__main__":
    print(encode_message("hello()"))
