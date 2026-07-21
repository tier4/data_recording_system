#!/usr/bin/env python3
import yaml
import sys
import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional

def load_yaml_file(filepath: Path) -> Optional[Dict[str, Any]]:
    """Load a YAML file

    Args:
        filepath: Path to the YAML file

    Returns:
        Dictionary of YAML data, None on error
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        logging.error(f"Error reading {filepath}: {e}")
        return None

def save_yaml_file(data: Dict[str, Any], filepath: Path) -> bool:
    """Save data to a YAML file

    Args:
        data: Data to save
        filepath: Path to save the file

    Returns:
        True on success, False on failure
    """
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        return True
    except Exception as e:
        logging.error(f"Error saving to {filepath}: {e}")
        return False

def merge_yaml_data(base_data: Optional[Dict], new_data: Optional[Dict]) -> Dict:
    """Recursively merge YAML data

    Args:
        base_data: Base data dictionary
        new_data: Data to merge

    Returns:
        Merged data dictionary
    """
    if base_data is None:
        return new_data or {}
    if new_data is None:
        return base_data

    if isinstance(new_data, dict) and isinstance(base_data, dict):
        result = base_data.copy()
        for key, value in new_data.items():
            if key in result:
                result[key] = merge_yaml_data(result[key], value)
            else:
                result[key] = value
        return result

    return new_data

class YamlAggregator:
    """Class for aggregating YAML files"""

    DEFAULT_OUTPUT = "multi_tf_static.yaml"

    BASE_LINK_DATA = {
        "base_link": {
            "drs_base_link": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0
            }
        }
    }


    def __init__(self, input_dir: str, output_file: str = DEFAULT_OUTPUT):
        self.input_path = Path(input_dir)
        self.output_path = Path(output_file)  # Store output path directly
        self.result_data = {}
        self.child_key_sources: Dict[str, Dict[str, List[str]]] = {}  # Track sources of child keys per parent key

    def validate_input_directory(self) -> bool:
        """Validate the input directory"""
        if not self.input_path.exists():
            logging.error(f"Directory {self.input_path} does not exist")
            return False
        if not self.input_path.is_dir():
            logging.error(f"{self.input_path} is not a directory")
            return False
        return True

    def get_yaml_files(self) -> List[Path]:
        """Get YAML files to aggregate"""
        # Exclude output file only if it's in the input directory
        output_name = self.output_path.name if self.output_path.parent == self.input_path else None
        return sorted([
            f for f in self.input_path.glob("*.yaml")
            if f.name != output_name and f.is_file()
        ])

    def detect_duplicate_child_keys(self, data: Dict, filename: str, parent_key: str = "") -> None:
        """Detect duplicate child keys under parent keys

        Args:
            data: Data to check
            filename: Source filename
            parent_key: Parent key name
        """
        if not isinstance(data, dict):
            return

        for key, value in data.items():
            # Treat top-level keys as parent keys
            if not parent_key:  # At top level
                if isinstance(value, dict):
                    # Track child keys under this parent key
                    if key not in self.child_key_sources:
                        self.child_key_sources[key] = {}

                    for child_key in value.keys():
                        if child_key not in self.child_key_sources[key]:
                            self.child_key_sources[key][child_key] = []
                        self.child_key_sources[key][child_key].append(filename)

                    # Process recursively (limit nesting depth to 2 levels)
                    self.detect_duplicate_child_keys(value, filename, key)

    def report_duplicates(self) -> None:
        """Report warnings for duplicate child keys"""
        has_duplicates = False
        warnings = []

        for parent_key, children in sorted(self.child_key_sources.items()):
            for child_key, sources in sorted(children.items()):
                if len(sources) > 1:
                    has_duplicates = True
                    warnings.append((parent_key, child_key, sources))

        if has_duplicates:
            logging.warning("Duplicate child keys detected:")
            for parent_key, child_key, sources in warnings:
                logging.warning(f"  Under '{parent_key}': key '{child_key}' found in multiple files:")
                for source in sorted(set(sources)):
                    count = sources.count(source)
                    if count > 1:
                        logging.warning(f"    - {source} ({count} times)")
                    else:
                        logging.warning(f"    - {source}")

    def load_and_merge_files(self, yaml_files: List[Path]) -> None:
        """Load and merge YAML files"""
        self.result_data = merge_yaml_data({}, self.BASE_LINK_DATA)
        self.child_key_sources.clear()

        # Also track keys from BASE_LINK_DATA
        self.detect_duplicate_child_keys(self.BASE_LINK_DATA, "<built-in>")

        for yaml_file in yaml_files:
            logging.info(f"Processing: {yaml_file.name}")
            data = load_yaml_file(yaml_file)
            if data:
                # Detect duplicate child keys
                self.detect_duplicate_child_keys(data, yaml_file.name)
                self.result_data = merge_yaml_data(self.result_data, data)
                logging.info(f"  - {data}")

        # Display duplicate key warnings
        self.report_duplicates()


    def write_output_file(self) -> bool:
        """Write results to output file"""
        output_path = self.output_path

        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                # Write header comments with timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                header_comments = [
                    "# Multi TF Static Publisher Configuration",
                    "# This file combines all calibration transforms for the DRS system",
                    f"# Generated: {timestamp}",
                    ""
                ]
                for comment in header_comments:
                    f.write(f"{comment}\n")

                # Write all data as a single YAML document
                yaml.dump(
                    self.result_data,
                    f,
                    default_flow_style=False,
                    sort_keys=False,
                    allow_unicode=True
                )

            print(f"Output saved to: {output_path}")
            logging.info(f"Output saved to: {output_path}")
            return True

        except Exception as e:
            logging.error(f"Error writing output file: {e}")
            return False

    def aggregate(self) -> bool:
        """Execute aggregation process"""
        if not self.validate_input_directory():
            return False

        yaml_files = self.get_yaml_files()
        if not yaml_files:
            logging.warning(f"No YAML files found in {self.input_path}")
            return False

        print(f"Found {len(yaml_files)} YAML files to aggregate")
        logging.info(f"Found {len(yaml_files)} YAML files:")
        for f in yaml_files:
            logging.info(f"  - {f.name}")

        self.load_and_merge_files(yaml_files)
        return self.write_output_file()


def aggregate_yaml_files(input_dir: str, output_file: str = "multi_tf_static.yaml") -> bool:
    """Aggregate all YAML files in the specified directory (wrapper for backward compatibility)

    Args:
        input_dir: Path to input directory
        output_file: Output filename

    Returns:
        True on success, False on failure
    """
    aggregator = YamlAggregator(input_dir, output_file)
    return aggregator.aggregate()

def setup_logging(verbose: bool = False) -> None:
    """Configure logging settings"""
    level = logging.DEBUG if verbose else logging.WARNING
    logging.basicConfig(
        level=level,
        format='%(levelname)s: %(message)s'
    )


def main() -> None:
    """Main function"""
    import argparse

    parser = argparse.ArgumentParser(
        prog="aggregate_calibration_files",
        description="Aggregate calibration YAML files for ROS static transforms"
    )
    parser.add_argument(
        "input_dir",
        nargs="?",
        default="data",
        help="Input directory containing YAML files (default: data)"
    )
    parser.add_argument(
        "output_file",
        nargs="?",
        default="multi_tf_static.yaml",
        help="Output file path (default: multi_tf_static.yaml in current directory)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--no-warnings",
        action="store_true",
        help="Suppress duplicate key warnings"
    )

    args = parser.parse_args()

    setup_logging(args.verbose)

    # Suppress duplicate warnings if requested
    if args.no_warnings:
        # Set to ERROR to suppress WARNING messages
        logging.getLogger().setLevel(logging.ERROR)

    success = aggregate_yaml_files(args.input_dir, args.output_file)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
