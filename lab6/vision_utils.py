import pandas as pd

def compute_metrics(csv_file, area_min=200):
    df = pd.read_csv(csv_file)
    total_frames = len(df)
    wrong_frames = df[(df['cx'] == -1) | (df['area'] < area_min)]
    num_wrong = len(wrong_frames)
    success_rate = (total_frames - num_wrong) / total_frames * 100
    print(f"Test: {csv_file}")
    print(f"Total frames: {total_frames}")
    print(f"Wrong frames: {num_wrong}")
    print(f"Success rate: {success_rate:.2f}%\n")
    return total_frames, num_wrong, success_rate

# Example
compute_metrics("results/results_log_varying_distance.csv")
