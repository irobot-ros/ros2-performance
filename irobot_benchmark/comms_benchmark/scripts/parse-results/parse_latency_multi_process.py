import pandas as pd
import sys

def generate_summary(csv_file):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file)

    # Extracting relevant columns
    df['Directory'] = df['Directory'].str.split('/').str[0]
    df['Category'] = df['Directory'].str.split('_').str[-1]
    df['Sub'] = df['AvgSubLatency']
    df = df[['Directory', 'Sub', 'AvgPubDuration']]

    # Pivot the DataFrame
    df_pivot = pd.pivot_table(df, values=['Sub', 'AvgPubDuration'], index='Directory', aggfunc='first')

    # Print the new DataFrame
    print(df_pivot.fillna(0))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py input_csv_file")
    else:
        csv_file = sys.argv[1]
        generate_summary(csv_file)
