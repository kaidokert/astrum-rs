// TODO: separation of concerns — this DRY-up refactoring of low32_buf is
// bundled with the blocking IPC backlog close for expedience; consider
// splitting into a separate commit in future cleanups.
/// Thread-safe, map-once allocator for fixed low-address pages.
///
/// On 64-bit test hosts, syscall dispatch casts `u32` register values to
/// pointers.  These pointers must resolve to real memory, so tests `mmap`
/// pages at deterministic low addresses.  Using `MAP_FIXED` every time is
/// racy: a concurrent test can remap (and zero) the page between a write
/// and its assertion.
///
/// `low32_buf(page)` guarantees each page index is mapped exactly once
/// (via `OnceLock`) and returns the stable pointer on every subsequent call.
use std::sync::OnceLock;

/// Maximum number of distinct page indices supported.
const MAX_PAGES: usize = 4;

static PAGES: [OnceLock<usize>; MAX_PAGES] = [
    OnceLock::new(),
    OnceLock::new(),
    OnceLock::new(),
    OnceLock::new(),
];

/// Return a pointer to a 4 KiB page at `0x2000_0000 + page * 0x1000`.
///
/// The page is mapped once (with `MAP_FIXED`) and cached; subsequent calls
/// with the same `page` return the cached pointer without re-mapping.
///
/// # Panics
///
/// Panics if `page >= MAX_PAGES` or if the underlying `mmap` fails.
pub fn low32_buf(page: usize) -> *mut u8 {
    assert!(page < MAX_PAGES, "page index {page} >= {MAX_PAGES}");
    let addr = *PAGES[page].get_or_init(|| {
        extern "C" {
            fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
        }
        let addr = 0x2000_0000 + page * 4096;
        // SAFETY: MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED at a known-free
        // low address.  The mapping is intentionally leaked (test-only).
        let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
        assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
        ptr as usize
    });
    addr as *mut u8
}
