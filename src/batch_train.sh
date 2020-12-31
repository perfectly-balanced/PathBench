#PBS -lwalltime=01:00:00
#PBS -lselect=1:ncpus=4:mem=48gb

module load anaconda3/personal

pip3 install -r $HOME/projects/PathBench/requirements.txt
python3 $HOME/projects/PathBench/src/run_trainer.py -n 30 -f
