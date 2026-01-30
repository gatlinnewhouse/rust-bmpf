# Ziggurat-rs: Fast Normal-, Exponential-, and Polynomial-Distributed Pseudo-Random Numbers (in Rust!)

While investigating Bayesian Particle Filtering, Bart Massey alerted me to some
code he wrote which improved upon the algorithm. In order to understand the
underlying concepts, I began by trying to port it to Rust. His code relied upon
his [Ziggurat](https://github.com/BartMassey/ziggurat/) code, so I had to start
by porting that to Rust.

I am sure some improvements are possible. I mainly used
[C2Rust](https://c2rust.com/) and some LLMs to provide input on how to convert
the code (although many LLMs make mistakes with Rust with respect to lifetimes,
`build.rs` scripts, and missing functionality).

## Performance comparison

In general the Rust version is slightly slower. This may be due to some errors
in my programming, I am sure there are opportunities for improvement.

A script is included that will compare the table generation and output of
`polytest` and `normaltest`. In general, the values generated are very close to
the C version, but slight differences exist in precision lengths.

Here are quick timing comparisons from `compare.sh` on my 2019 Macbook Pro
(16in) which has a Intel Core i9-9980HK at 5GHz with 8 cores and 64GB of RAM:

### Polytest
C version:
```
real    0m0.496s
user    0m0.458s
sys     0m0.033s
```

Rust version:
```
real    0m0.683s
user    0m0.596s
sys     0m0.074s
```

### Normaltest
C version:
```
real    0m0.151s
user    0m0.130s
sys     0m0.018s
```

Rust version:
```
real    0m0.321s
user    0m0.224s
sys     0m0.074s
```
