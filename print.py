import os
import csv

def export_tree_to_csv(startpath, output_file):
    with open(output_file, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        # Headers for your CSV
        writer.writerow(['Type', 'Name', 'Path', 'Level'])
        
        for root, dirs, files in os.walk(startpath):
            # Calculate depth level
            level = root.replace(startpath, '').count(os.sep)
            
            # Write the Directory entry
            writer.writerow(['Folder', os.path.basename(root), root, level])
            
            # Write the File entries
            for f in files:
                file_path = os.path.join(root, f)
                writer.writerow(['File', f, file_path, level + 1])

# Run the function
export_tree_to_csv('.', 'folder_structure.csv')
print("✅ Folder structure exported to folder_structure.csv")