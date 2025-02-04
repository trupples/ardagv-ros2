cd ../../ 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cd src/adi_3dtof_adtf31xx
for file in $(find ../../build -name compile_commands.json) ; do
	run-clang-tidy -fix -p $(dirname $file)
done
