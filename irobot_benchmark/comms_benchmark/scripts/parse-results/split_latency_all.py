import os
import sys

def extract_sections(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    subscriptions_started = False
    publishers_started = False
    sub_lines = []
    pub_lines = []

    for line in lines:
        if "Subscriptions stats:" in line:
            subscriptions_started = True
            continue
        elif "Publishers stats:" in line:
            publishers_started = True
            subscriptions_started = False
            continue
        elif subscriptions_started:
            if line.strip():
                sub_lines.append(line)
        elif publishers_started:
            if line.strip():
                pub_lines.append(line)

    return sub_lines, pub_lines

def process_directory(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file == 'latency_all.txt':
                file_path = os.path.join(root, file)
                sub_lines, pub_lines = extract_sections(file_path)
                save_to_csv(sub_lines, os.path.join(root, "sub_latency.csv"))
                save_to_csv(pub_lines, os.path.join(root, "pub_duration.csv"))

def save_to_csv(lines, filename):
    with open(filename, 'w') as f:
        f.writelines(lines)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <directory>")
        sys.exit(1)

    directory = sys.argv[1]
    if not os.path.isdir(directory):
        print("Error: Directory '{}' not found!".format(directory))
        sys.exit(1)

    process_directory(directory)
