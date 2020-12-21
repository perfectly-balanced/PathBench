#PBS -lwalltime=01:00:00
#PBS -lselect=1:ncpus=4:mem=48gb

module load anaconda3/personal

pip3 install -r $HOME/projects/PathBench/trainer_requirements.txt
python3 $HOME/projects/PathBench/src/runtrainer.py

mkdir $WORK/$PBS_JOBID
cp * $WORK/$PBS_JOBID
