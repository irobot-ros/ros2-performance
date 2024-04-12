import csv
import sys

def process_csv(input_file):
    data = {}
    with open(input_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            directory = row['Directory']
            name, comm = directory.split('/')[-2:]
            average_cpu = row['Average_CPU']
            average_rss_mb = row['Average_RSS_MB']
            average_vsz_mb = row['Average_VSZ_MB']
            if name not in data:
                data[name] = {}
            data[name][comm] = {'CPU': average_cpu, 'RSS': average_rss_mb, 'VSZ': average_vsz_mb}

    return data

def print_output(data):
    name_list = sorted(data.keys())
    comm_set = set()
    for name in name_list:
        for comm in data[name]:
            comm_set.add(comm)

    for metric in ['CPU', 'RSS', 'VSZ']:
        print('Topology,' + ','.join(sorted(comm_set)))
        for name in name_list:
            row = [metric + '_' + name] + [data[name][comm].get(metric, '') for comm in sorted(comm_set)]
            print(','.join(row))
        print()  # Print an empty line between different metrics

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python parse_averages_single_process.py average_metrics.csv")
        sys.exit(1)

    input_file = sys.argv[1]
    data = process_csv(input_file)
    print_output(data)
