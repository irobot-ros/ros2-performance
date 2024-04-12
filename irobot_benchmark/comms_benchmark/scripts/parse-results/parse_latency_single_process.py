import pandas as pd
import sys

def generate_csv_tables(vcs_file):
    # Read the VCS file into a DataFrame
    df = pd.read_csv(vcs_file)

    # Split the 'Directory' column to extract topology and type
    df[['Topology', 'Type']] = df['Directory'].str.split('/', expand=True)

    # Pivot the DataFrame to rearrange data for AvgSubLatency
    avg_sub_latency_df = df.pivot(index='Type', columns='Topology', values='AvgSubLatency')

    # Pivot the DataFrame to rearrange data for AvgPubDuration
    avg_pub_duration_df = df.pivot(index='Type', columns='Topology', values='AvgPubDuration')

    # Transpose the DataFrames
    transposed_avg_sub_latency_df = avg_sub_latency_df.T
    transposed_avg_pub_duration_df = avg_pub_duration_df.T

    # Print the transposed CSV tables to console
    print("Transposed AvgSubLatency Table:")
    print(transposed_avg_sub_latency_df.to_csv(sep=","))
    print("\nTransposed AvgPubDuration Table:")
    print(transposed_avg_pub_duration_df.to_csv(sep=","))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py input.vcs")
        sys.exit(1)
    
    vcs_file = sys.argv[1]
    generate_csv_tables(vcs_file)
