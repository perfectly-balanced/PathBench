#PBS -lwalltime=06:00:00
#PBS -lselect=1:ncpus=16:mem=96gb:ngpus=4:gpu_type=RTX6000

python3 runtrainer.py