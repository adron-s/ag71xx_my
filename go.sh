make clean
make || exit 1
echo
echo "All done. Do NC -> LEDE host"
#cat ./ag71xx.ko | nc -l -p 1234 -q 1
sleep 1
make clean
