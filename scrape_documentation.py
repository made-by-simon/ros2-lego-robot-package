import os
import re
from pathlib import Path

def clean_rst_content(content):
    """
    Remove RST formatting and normalize line breaks.
    
    Args:
        content: RST file content as string
    
    Returns:
        Cleaned content string
    """
    # Remove RST directives (.. directive::)
    content = re.sub(r'\.\.\s+\w+::[^\n]*\n', '', content)
    
    # Remove RST options (lines starting with :option:)
    content = re.sub(r'^\s*:\w+:.*$', '', content, flags=re.MULTILINE)
    
    # Remove RST code block markers (::)
    content = re.sub(r'::\s*\n', '\n', content)
    
    # Remove RST headers (lines of ===, ---, ~~~, etc.)
    content = re.sub(r'^[=\-~`#*^"+]{3,}$', '', content, flags=re.MULTILINE)
    
    # Remove RST inline markup
    content = re.sub(r':[\w-]+:`([^`]+)`', r'\1', content)  # :role:`text`
    content = re.sub(r'`([^`]+)`_', r'\1', content)  # `link`_
    content = re.sub(r'\*\*([^*]+)\*\*', r'\1', content)  # **bold**
    content = re.sub(r'\*([^*]+)\*', r'\1', content)  # *italic*
    content = re.sub(r'``([^`]+)``', r'\1', content)  # ``code``
    
    # Remove RST comments (.. followed by comment text)
    content = re.sub(r'\.\.\s+[^\n]*\n(?:\s+[^\n]*\n)*', '', content)
    
    # Remove reference targets (.. _target:)
    content = re.sub(r'\.\.\s+_[\w-]+:', '', content)
    
    # Remove multiple consecutive blank lines
    content = re.sub(r'\n{3,}', '\n\n', content)
    
    # Remove leading/trailing whitespace from each line
    lines = [line.rstrip() for line in content.split('\n')]
    content = '\n'.join(lines)
    
    # Remove excessive blank lines again after line cleanup
    content = re.sub(r'\n{3,}', '\n\n', content)
    
    return content.strip()

def resolve_path(base_file, referenced_path):
    """
    Resolve a referenced path relative to the base file.
    
    Args:
        base_file: Path object of the file containing the reference
        referenced_path: String path referenced in the directive
    
    Returns:
        Resolved Path object or None if not found
    """
    base_dir = base_file.parent
    
    # Try relative to base file
    resolved = (base_dir / referenced_path).resolve()
    if resolved.exists():
        return resolved
    
    # Try with common extensions if no extension provided
    if not resolved.suffix:
        for ext in ['.rst', '.py', '.txt', '.md']:
            with_ext = resolved.with_suffix(ext)
            if with_ext.exists():
                return with_ext
    
    return None

def extract_references(content, base_file):
    """
    Extract file references from Sphinx directives.
    
    Args:
        content: RST file content as string
        base_file: Path object of the file being parsed
    
    Returns:
        List of resolved Path objects
    """
    references = []
    
    # Match .. include:: and .. literalinclude:: directives
    include_pattern = r'\.\.\s+(?:include|literalinclude)::\s+(.+?)(?:\s|$)'
    includes = re.findall(include_pattern, content, re.MULTILINE)
    
    for ref in includes:
        ref = ref.strip()
        resolved = resolve_path(base_file, ref)
        if resolved and resolved not in references:
            references.append(resolved)
    
    # Match .. toctree:: entries
    toctree_pattern = r'\.\.\s+toctree::.*?\n((?:\s+.+\n)*)'
    toctrees = re.findall(toctree_pattern, content, re.MULTILINE | re.DOTALL)
    
    for toctree_block in toctrees:
        # Extract non-empty, non-option lines (options start with :)
        lines = toctree_block.split('\n')
        for line in lines:
            line = line.strip()
            if line and not line.startswith(':'):
                resolved = resolve_path(base_file, line)
                if resolved and resolved not in references:
                    references.append(resolved)
    
    return references

def parse_rst_files(root_dir, output_file):
    """
    Recursively find all .rst files and combine their content into one text file.
    Follows Sphinx references to include referenced files.
    Removes RST formatting and normalizes line breaks.
    
    Args:
        root_dir: Root directory to start searching from
        output_file: Output text file path
    """
    root_path = Path(root_dir)
    
    # Find all .rst files recursively
    rst_files = list(root_path.rglob("*.rst"))
    
    print(f"Found {len(rst_files)} .rst files")
    
    # Track all files we've processed (including references)
    processed_files = set()
    all_referenced_files = set()
    
    # First pass: collect all references
    print("\nScanning for referenced files...")
    for rst_file in rst_files:
        try:
            with open(rst_file, 'r', encoding='utf-8') as f:
                content = f.read()
                refs = extract_references(content, rst_file)
                all_referenced_files.update(refs)
        except Exception as e:
            print(f"Error scanning {rst_file.name}: {e}")
    
    # Filter out .rst files (already in main list) and non-existent files
    additional_files = [f for f in all_referenced_files 
                       if f.suffix != '.rst' and f.exists()]
    
    print(f"Found {len(additional_files)} additional referenced files")
    
    # Open output file and write all content
    with open(output_file, 'w', encoding='utf-8') as outfile:
        # Write main RST files
        outfile.write("="*80 + "\n")
        outfile.write("RST DOCUMENTATION FILES\n")
        outfile.write("="*80 + "\n\n")
        
        for rst_file in sorted(rst_files):
            relative_path = rst_file.relative_to(root_path)
            separator = f"\n{'='*80}\n"
            separator += f"File: {relative_path}\n"
            separator += f"{'='*80}\n\n"
            
            outfile.write(separator)
            
            try:
                with open(rst_file, 'r', encoding='utf-8') as infile:
                    content = infile.read()
                    # Clean RST formatting
                    cleaned_content = clean_rst_content(content)
                    outfile.write(cleaned_content)
                    outfile.write("\n\n")
                
                processed_files.add(rst_file)
                print(f"Processed: {relative_path}")
                
            except Exception as e:
                error_msg = f"Error reading {relative_path}: {str(e)}\n\n"
                outfile.write(error_msg)
                print(error_msg.strip())
        
        # Write referenced files
        if additional_files:
            outfile.write("\n\n" + "="*80 + "\n")
            outfile.write("REFERENCED FILES (includes, code examples, etc.)\n")
            outfile.write("="*80 + "\n\n")
            
            for ref_file in sorted(additional_files):
                try:
                    relative_path = ref_file.relative_to(root_path)
                except ValueError:
                    # File is outside root directory
                    relative_path = ref_file
                
                separator = f"\n{'='*80}\n"
                separator += f"File: {relative_path}\n"
                separator += f"{'='*80}\n\n"
                
                outfile.write(separator)
                
                try:
                    with open(ref_file, 'r', encoding='utf-8') as infile:
                        content = infile.read()
                        # Only clean if it's an RST-like file
                        if ref_file.suffix in ['.rst', '.txt', '.md']:
                            content = clean_rst_content(content)
                        outfile.write(content)
                        outfile.write("\n\n")
                    
                    print(f"Processed reference: {relative_path}")
                    
                except Exception as e:
                    error_msg = f"Error reading {relative_path}: {str(e)}\n\n"
                    outfile.write(error_msg)
                    print(error_msg.strip())
    
    print(f"\nTotal files processed: {len(processed_files) + len(additional_files)}")
    print(f"Output saved to: {output_file}")

if __name__ == "__main__":
    # Configuration
    docs_directory = "C:/Users/Simon/Desktop/pybricks-api/doc/main"  # Current directory, change as needed
    output_filename = "pybricks_docs_combined.txt"
    
    # Run parser
    parse_rst_files(docs_directory, output_filename)