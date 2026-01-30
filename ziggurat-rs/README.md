# Ziggurat-rs: Fast Normal-, Exponential-, and Polynomial-Distributed Pseudo-Random Numbers (in Rust!)

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
