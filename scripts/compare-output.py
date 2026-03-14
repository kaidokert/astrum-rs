#!/usr/bin/env python3
"""Compare QEMU output against an expected file.
First-line directive selects mode: exact (default), regex, or contains.
Exit: 0=match, 1=mismatch, 2=usage error.
"""
import difflib, re, sys


def parse_mode(lines):
    if lines:
        m = re.match(r"^#\s*mode:\s*(\w+)", lines[0])
        if m:
            mode = m.group(1)
            return (mode, lines[1:]) if mode in ("exact", "regex", "contains") else (None, lines)
    return "exact", lines


def compare(expected_text, actual_text):
    exp_lines = expected_text.splitlines()
    mode, exp_body = parse_mode(exp_lines)
    if mode is None:
        return False, "error: unknown mode directive in expected file"
    act_lines = actual_text.splitlines()
    if mode == "exact":
        if exp_body == act_lines:
            return True, ""
        diff = difflib.unified_diff(exp_body, act_lines, fromfile="expected", tofile="actual", lineterm="")
        return False, "\n".join(diff)
    if mode == "regex":
        # Anchored sequence: each pattern must match an actual line at or after
        # the previous match position, preserving order but tolerating extra lines.
        fails = []
        pos = 0
        for i, pat in enumerate(exp_body, 1):
            matched = False
            for j in range(pos, len(act_lines)):
                try:
                    if re.fullmatch(pat, act_lines[j]):
                        pos = j + 1
                        matched = True
                        break
                except re.error as e:
                    fails.append(f"pattern {i}: bad regex '{pat}': {e}")
                    matched = True  # skip further search for broken pattern
                    break
            if not matched:
                fails.append(f"pattern {i}: no match for /{pat}/ in remaining output (from line {pos + 1})")
        return (True, "") if not fails else (False, "\n".join(fails))
    # contains: ordered subsequence match — each expected substring must appear
    # in an actual line at or after the previous match position.
    fails = []
    pos = 0
    for i, s in enumerate(exp_body, 1):
        matched = False
        for j in range(pos, len(act_lines)):
            if s in act_lines[j]:
                pos = j + 1
                matched = True
                break
        if not matched:
            fails.append(f"pattern {i} not found in order: '{s}'")
    return (True, "") if not fails else (False, "\n".join(fails))


def self_test():
    passed = failed = 0
    def check(name, ok):
        nonlocal passed, failed
        if ok: passed += 1
        else: failed += 1; print(f"FAIL: {name}")
    # exact mode (default)
    ok, _ = compare("hello\nworld", "hello\nworld"); check("exact-match", ok)
    ok, r = compare("hello\nworld", "hello\nearth"); check("exact-mismatch", not ok)
    check("exact-has-diff", "expected" in r and "actual" in r)
    # exact mode (explicit)
    ok, _ = compare("# mode: exact\nfoo", "foo"); check("exact-explicit-match", ok)
    ok, r = compare("# mode: exact\nfoo", "bar"); check("exact-explicit-mismatch", not ok)
    check("exact-diff-content", "-foo" in r and "+bar" in r)
    # regex mode — anchored sequence (tolerates extra lines)
    ok, _ = compare("# mode: regex\nhel+o\nw..ld", "hello\nworld"); check("regex-match", ok)
    ok, r = compare("# mode: regex\nhel+o\nw..ld", "hello\nearth"); check("regex-mismatch", not ok)
    check("regex-report", "pattern 2" in r)
    ok, _ = compare("# mode: regex\nhello", "hello\nextra"); check("regex-extra-ok", ok)
    ok, r = compare("# mode: regex\nhello\nworld", "hello"); check("regex-missing", not ok)
    check("regex-missing-msg", "no match" in r)
    ok, _ = compare("# mode: regex\nhello\nworld", "boot\nhello\ndebug\nworld\ndone"); check("regex-skip", ok)
    # contains mode — ordered subsequence
    ok, _ = compare("# mode: contains\nhello\nworld", "say hello!\nworld ok"); check("contains-match", ok)
    ok, r = compare("# mode: contains\nmissing", "hello\nworld"); check("contains-mismatch", not ok)
    check("contains-report", "missing" in r)
    ok, _ = compare("# mode: contains\nfoo", "something foo something"); check("contains-substr", ok)
    ok, r = compare("# mode: contains\nworld\nhello", "say hello!\nworld ok"); check("contains-order", not ok)
    # unknown mode
    ok, r = compare("# mode: bogus\nfoo", "foo"); check("unknown-mode", not ok)
    check("unknown-mode-msg", "unknown mode" in r)
    print(f"\n{passed} passed, {failed} failed")
    return 0 if failed == 0 else 1


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--self-test":
        sys.exit(self_test())
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <expected> <actual>", file=sys.stderr); sys.exit(2)
    for path in sys.argv[1:3]:
        try:
            with open(path) as f:
                f.read()  # validate readable
        except OSError as e:
            print(f"error: {e}", file=sys.stderr); sys.exit(2)
    with open(sys.argv[1]) as ef, open(sys.argv[2]) as af:
        ok, report = compare(ef.read(), af.read())
    if not ok: print(report)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
