import pandas as pd
import sys

def process_csv(csv_file):
    # Read the CSV file
    df = pd.read_csv(csv_file)

    # Initialize dictionary to store results
    output_data_cpu = {}
    output_data_rss = {}
    output_data_vsz = {}

    # Iterate over each row
    for index, row in df.iterrows():
        directory = row['Directory']
        size, comm, topology = directory.split('/')[:3]
        key = f"{size}/{comm}"
        pub_cpu = 0
        sub_cpu = 0
        pub_sub_cpu = 0
        debug_cpu = 0
        main_cpu = 0
        pub_rss = 0
        sub_rss = 0
        pub_sub_rss = 0
        debug_rss = 0
        main_rss = 0
        pub_vsz = 0
        sub_vsz = 0
        pub_sub_vsz = 0
        debug_vsz = 0
        main_vsz = 0

        if 'debug' in topology:
            debug_cpu = row['Average_CPU']
            debug_rss = row['Average_RSS_MB']
            debug_vsz = row['Average_VSZ_MB']
        elif 'pub' in topology:
            if 'pub_sub' in topology:
                pub_sub_cpu = row['Average_CPU']
                pub_sub_rss = row['Average_RSS_MB']
                pub_sub_vsz = row['Average_VSZ_MB']
            else:
                pub_cpu = row['Average_CPU']
                pub_rss = row['Average_RSS_MB']
                pub_vsz = row['Average_VSZ_MB']
        elif 'sub' in topology:
            sub_cpu = row['Average_CPU']
            sub_rss = row['Average_RSS_MB']
            sub_vsz = row['Average_VSZ_MB']
        else:
            main_cpu = row['Average_CPU']
            main_rss = row['Average_RSS_MB']
            main_vsz = row['Average_VSZ_MB']

        if key not in output_data_cpu:
            output_data_cpu[key] = {'pub_cpu': 0, 'sub_cpu': 0, 'pub_sub_cpu': 0, 'debug_cpu': 0, 'main_cpu': 0}
        output_data_cpu[key]['pub_cpu'] += pub_cpu
        output_data_cpu[key]['sub_cpu'] += sub_cpu
        output_data_cpu[key]['pub_sub_cpu'] += pub_sub_cpu
        output_data_cpu[key]['debug_cpu'] += debug_cpu
        output_data_cpu[key]['main_cpu'] += main_cpu

        if key not in output_data_rss:
            output_data_rss[key] = {'pub_rss': 0, 'sub_rss': 0, 'pub_sub_rss': 0, 'debug_rss': 0, 'main_rss': 0}
        output_data_rss[key]['pub_rss'] += pub_rss
        output_data_rss[key]['sub_rss'] += sub_rss
        output_data_rss[key]['pub_sub_rss'] += pub_sub_rss
        output_data_rss[key]['debug_rss'] += debug_rss
        output_data_rss[key]['main_rss'] += main_rss

        if key not in output_data_vsz:
            output_data_vsz[key] = {'pub_vsz': 0, 'sub_vsz': 0, 'pub_sub_vsz': 0, 'debug_vsz': 0, 'main_vsz': 0}
        output_data_vsz[key]['pub_vsz'] += pub_vsz
        output_data_vsz[key]['sub_vsz'] += sub_vsz
        output_data_vsz[key]['pub_sub_vsz'] += pub_sub_vsz
        output_data_vsz[key]['debug_vsz'] += debug_vsz
        output_data_vsz[key]['main_vsz'] += main_vsz

    # Initialize lists to store combined data for CPU, RSS, and VSZ
    combined_data_cpu = []
    combined_data_rss = []
    combined_data_vsz = []

    # Combine data into lists for CPU, RSS, and VSZ
    for key, value in output_data_cpu.items():
        total_cpu = value['pub_cpu'] + value['sub_cpu'] + value['pub_sub_cpu'] + value['debug_cpu'] + value['main_cpu']
        combined_data_cpu.append([key, value['pub_cpu'], value['sub_cpu'], value['pub_sub_cpu'], value['debug_cpu'], value['main_cpu'], total_cpu])

    for key, value in output_data_rss.items():
        total_rss = value['pub_rss'] + value['sub_rss'] + value['pub_sub_rss'] + value['debug_rss'] + value['main_rss']
        combined_data_rss.append([key, value['pub_rss'], value['sub_rss'], value['pub_sub_rss'], value['debug_rss'], value['main_rss'], total_rss])

    for key, value in output_data_vsz.items():
        total_vsz = value['pub_vsz'] + value['sub_vsz'] + value['pub_sub_vsz'] + value['debug_vsz'] + value['main_vsz']
        combined_data_vsz.append([key, value['pub_vsz'], value['sub_vsz'], value['pub_sub_vsz'], value['debug_vsz'], value['main_vsz'], total_vsz])

    # Create DataFrames for the output CSVs for CPU, RSS, and VSZ
    output_df_cpu = pd.DataFrame(combined_data_cpu, columns=['Dir', 'pub_cpu', 'sub_cpu', 'pub_sub_cpu', 'debug_cpu', 'main_cpu', 'total'])
    output_df_rss = pd.DataFrame(combined_data_rss, columns=['Dir', 'pub_rss', 'sub_rss', 'pub_sub_rss', 'debug_rss', 'main_rss', 'total'])
    output_df_vsz = pd.DataFrame(combined_data_vsz, columns=['Dir', 'pub_vsz', 'sub_vsz', 'pub_sub_vsz', 'debug_vsz', 'main_vsz', 'total'])

    # Print the DataFrames as comma-separated values
    print(output_df_cpu.to_csv(index=False))
    print(output_df_rss.to_csv(index=False))
    print(output_df_vsz.to_csv(index=False))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <input_csv_file>")
        sys.exit(1)

    input_csv_file = sys.argv[1]
    process_csv(input_csv_file)
