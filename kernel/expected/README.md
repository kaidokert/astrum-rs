# Expected Output Files

Each file in this directory corresponds to a QEMU example and contains its
expected semihosting stdout. The runner compares captured output against these
files to detect regressions.

## Format

The first line is a **mode directive** that controls how the comparison is
performed:

    exact   — captured output must match the file byte-for-byte (ignoring
              a trailing newline)
    contains — every non-directive, non-blank line must appear somewhere in
               the captured output (order-independent)

Lines starting with `#` are comments and are ignored.

### Example (`expected/hello.out`)

```
exact
Hello from the RTOS kernel!
ALL TESTS PASSED
```
