import os
import pandas as pd

def transform_csv(csv_path, csvf):
	script_dir = os.path.dirname(os.path.abspath(__file__))
	parent_dir = os.path.dirname(script_dir)
	csv_path = os.path.join(parent_dir, csv_path)
	full_csv_path = os.path.join(csv_path, csvf)
	
	velocity_sets = []
	
	df = pd.read_csv(full_csv_path)
	
	for _, row in df.iterrows():
		motion_dict = {'linear' : list(), 'angular' : list()}
		motion_dict['linear'].extend([row['ax'], row['ay'], row['az']])
		motion_dict['angular'].extend([row['wx'], row['wy'], row['wz']])
		
		velocity_sets.append(motion_dict)
	
	return velocity_sets
	
