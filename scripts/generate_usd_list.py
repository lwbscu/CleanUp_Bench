import os

def get_file_size(file_path):
    """Returns the file size in a human-readable format."""
    size_in_bytes = os.path.getsize(file_path)
    if size_in_bytes < 1024:
        return f"{size_in_bytes} B"
    elif size_in_bytes < 1024**2:
        return f"{size_in_bytes/1024:.2f} KB"
    elif size_in_bytes < 1024**3:
        return f"{size_in_bytes/1024**2:.2f} MB"
    else:
        return f"{size_in_bytes/1024**3:.2f} GB"

def find_usd_files(root_dir, output_file):
    """
    Finds all .usd files in a directory and writes their absolute and relative paths to a markdown file.
    """
    workspace_root = '/home/lwb/isaacsim/extension_examples'
    with open(output_file, 'w') as f:
        f.write("# USD File List (CleanUp_Bench)\n\n")
        f.write("This document lists all `.usd` files found within the `CleanUp_Bench` directory, along with their absolute and relative paths and file sizes.\n\n")
        f.write("---\n\n")

        usd_files_found = False
        for dirpath, _, filenames in os.walk(root_dir):
            for filename in filenames:
                if filename.endswith('.usd'):
                    usd_files_found = True
                    absolute_path = os.path.abspath(os.path.join(dirpath, filename))
                    relative_path = os.path.relpath(absolute_path, workspace_root)
                    file_size = get_file_size(absolute_path)
                    
                    f.write(f"- **File:** `{filename}`\n")
                    f.write(f"  - **Absolute Path:** `{absolute_path}`\n")
                    f.write(f"  - **Relative Path:** `{relative_path}`\n")
                    f.write(f"  - **Size:** `{file_size}`\n\n")

        if not usd_files_found:
            f.write("No `.usd` files were found in the specified directory.\n")

if __name__ == "__main__":
    # The script is in CleanUp_Bench/scripts, so we go up two levels to the workspace root
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    search_directory = os.path.join(workspace_root, 'CleanUp_Bench')
    output_md_file = os.path.join(workspace_root, 'usd_file_list.md')

    print(f"Searching for .usd files in: {search_directory}")
    find_usd_files(search_directory, output_md_file)
    print(f"Markdown file generated at: {output_md_file}")
