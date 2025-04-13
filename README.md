# CompArch---prog-8
In a real processor, the MEMORY stage is heavily affected by cache performance, and specifically by the occurrence of cache misses. When the CPU attempts to access memory during load or store operations, it first checks if the data is available in the cache (a smaller, faster memory located closer to the CPU). If the data is present (a cache hit), access is fast. However, if the data is not present in the cache (a cache miss), the processor must fetch it from main memory (RAM), which is significantly slower — sometimes by an order of magnitude or more.

In the context of this Verilog-based Tinker CPU, while there's no cache implemented in the current design, you can think of every memory access as effectively behaving like a cache miss, since all reads and writes go directly to the large bytes[] array. This further supports the reasoning that the MEMORY stage is the slowest, because in a real system, repeated cache misses would stall the pipeline or force it to wait until memory access is completed.

The alu may also take a long time in a real cpu with operations such as division taking multiple cycles

euid: cvt372
