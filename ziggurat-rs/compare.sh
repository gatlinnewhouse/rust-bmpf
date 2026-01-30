#!/bin/bash
cargo build --release
cargo build --release --example normaltest
cargo build --release --example polytest

cd ziggurat-c
make check
cd ..

# c normal table:
cat ziggurat-c/normal_tab.c | rg -o "[0-9]+U" | rg -o "[0-9]+" >c-norms_k.tmp
cat ziggurat-c/normal_tab.c | rg -o "[0-9]+e\-[0-9]+" >c-norms_w.tmp
sed -i '' 's/09$/9/g' c-norms_w.tmp
cat ziggurat-c/normal_tab.c | rg -o "[0-9\.]+" | rg -o "[0-9\.]+" | tail -n 256 >c-norms_f.tmp

# rust normal table:
cat src/tables/normal.rs | rg "[0-9]+," | head -n 256 | rg -o "[0-9]+" >rs-norms_k.tmp
cat src/tables/normal.rs | rg -o "[0-9]+e\-[0-9]+" >rs-norms_w.tmp
cat src/tables/normal.rs | rg -o "[0-9\.]+," | rg -o "[0-9\.]+" | tail -n 256 >rs-norms_f.tmp

echo "Diffing Normal K tables..."
diff --side-by-side c-norms_k.tmp rs-norms_k.tmp
echo "Done!"
rm c-norms_k.tmp
rm rs-norms_k.tmp

echo "Diffing Normal W tables..."
diff --side-by-side c-norms_w.tmp rs-norms_w.tmp
echo "Done!"
rm c-norms_w.tmp
rm rs-norms_w.tmp

echo "Diffing Normal F tables..."
diff --side-by-side c-norms_f.tmp rs-norms_f.tmp
echo "Done!"
rm c-norms_f.tmp
rm rs-norms_f.tmp

# c exponential table:
cat ziggurat-c/exponential_tab.c | rg -o "[0-9]+U" | rg -o "[0-9]+" >c-exps_k.tmp
cat ziggurat-c/exponential_tab.c | rg -o "[0-9]+e\-[0-9]+" >c-exps_w.tmp
sed -i '' 's/09$/9/g' c-exps_w.tmp
cat ziggurat-c/exponential_tab.c | rg -o "[0-9\.]+" | rg -o "[0-9\.]+" | tail -n 256 >c-exps_f.tmp

# rust exponential table:
cat src/tables/exponential.rs | rg "[0-9]+," | head -n 256 | rg -o "[0-9]+" >rs-exps_k.tmp
cat src/tables/exponential.rs | rg -o "[0-9]+e\-[0-9]+" >rs-exps_w.tmp
cat src/tables/exponential.rs | rg -o "[0-9\.]+," | rg -o "[0-9\.]+" | tail -n 256 >rs-exps_f.tmp

echo "Diffing Exponential K tables..."
diff --side-by-side c-exps_k.tmp rs-exps_k.tmp
echo "Done!"
rm c-exps_k.tmp
rm rs-exps_k.tmp

echo "Diffing Exponential W tables..."
diff --side-by-side c-exps_w.tmp rs-exps_w.tmp
echo "Done!"
rm c-exps_w.tmp
rm rs-exps_w.tmp

echo "Diffing Exponential F tables..."
diff --side-by-side c-exps_f.tmp rs-exps_f.tmp
echo "Done!"
rm c-exps_f.tmp
rm rs-exps_f.tmp

time ../target/release/examples/polytest >polytest-rs.dat
echo "Diffing Polytest data"
diff --side-by-side ziggurat-c/polytest.dat polytest-rs.dat
echo "Done!"
gnuplot -p -e "plot 'polytest-rs.dat' using 1:2 with boxes, '' using 1:3 with lines" &

time ../target/release/examples/normaltest >normaltest-rs.dat
echo "Diffing Normaltest data"
diff --side-by-side ziggurat-c/normaltest.dat normaltest-rs.dat
echo "Done!"
gnuplot -p -e "plot 'normaltest-rs.dat' using 1:2 with boxes, '' using 1:3 with lines" &
