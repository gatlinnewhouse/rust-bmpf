#!/bin/sh
cargo run --release --example bpf -- \
  --nparticles 1000 --sampler "$1" --sort \
  --file bmpf-c/vehicle.dat >vehicle.out &&
gnuplot -p -c bpf.gnuplot
