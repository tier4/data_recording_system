#!/usr/bin/env python3
"""
DRS configuration file backup script

Retrieves files from specified folders on ecu0 and ecu1, then compresses and saves them.
"""

import argparse
import logging
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from zipfile import ZipFile
import shutil


class DRSBackup:
    """DRS configuration file backup class"""

    # ECU information
    ECU0_HOST = "192.168.20.1"
    ECU1_HOST = "192.168.20.2"
    
    # Backup target directories
    BACKUP_DIRS = [
        "/opt/drs/config",
        "/opt/drs/service",
        "/opt/drs/install"
    ]
    
    DEFAULT_KEYWORD = "drs_backup"
    
    def __init__(self, keyword: str = DEFAULT_KEYWORD, user: str = None, use_scp: bool = False):
        """
        Initialize backup instance
        
        Args:
            keyword: Keyword for backup folder name
            user: SSH username (None means current user)
            use_scp: Use scp if True, rsync if False
        """
        self.keyword = keyword
        self.user = user
        self.use_scp = use_scp
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.backup_dir = Path(f"{self.timestamp}_{self.keyword}")
        
    def setup_logging(self, verbose: bool = False) -> None:
        """Configure logging"""
        level = logging.DEBUG if verbose else logging.INFO
        logging.basicConfig(
            level=level,
            format='%(levelname)s: %(message)s'
        )
    
    def build_remote_path(self, host: str, remote_path: str) -> str:
        """
        Build remote path string
        
        Args:
            host: Remote host address
            remote_path: Remote path
            
        Returns:
            Remote path string
        """
        if self.user:
            return f"{self.user}@{host}:{remote_path}"
        return f"{host}:{remote_path}"
    
    def backup_from_host(self, host: str, hostname: str) -> bool:
        """
        Backup files from specified host
        
        Args:
            host: Remote host IP address
            hostname: Hostname (ecu0/ecu1)
            
        Returns:
            True on success, False on failure
        """
        host_backup_dir = self.backup_dir / hostname
        
        logging.info(f"Backing up from {hostname} ({host})...")
        
        # Create directory for this host
        host_backup_dir.mkdir(parents=True, exist_ok=True)
        
        success = True
        for remote_dir in self.BACKUP_DIRS:
            logging.info(f"  Copying {remote_dir}...")
            
            # Create local target directory (preserve remote path structure)
            local_dir_name = Path(remote_dir).name
            local_target_dir = host_backup_dir / local_dir_name
            
            if self.use_scp:
                # Use scp
                result = self._copy_with_scp(host, remote_dir, local_target_dir)
            else:
                # Use rsync (default)
                result = self._copy_with_rsync(host, remote_dir, local_target_dir)
            
            if not result:
                logging.error(f"  Failed to copy {remote_dir}")
                success = False
            else:
                logging.info(f"  Successfully copied {remote_dir}")
        
        return success
    
    def _copy_with_rsync(self, host: str, remote_path: str, local_target: Path) -> bool:
        """
        Copy files using rsync
        
        Args:
            host: Remote host address
            remote_path: Remote path
            local_target: Local destination path
            
        Returns:
            True on success
        """
        remote = self.build_remote_path(host, remote_path)
        
        # Build rsync command
        # -a: Archive mode (preserve permissions, timestamps, etc.)
        # -v: Verbose output
        # -z: Compress during transfer
        # --delete: Sync deleted files
        cmd = [
            "rsync",
            "-avz",
            "--delete",
            f"{remote}/",
            str(local_target)
        ]
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True
            )
            if result.stdout:
                logging.debug(result.stdout)
            return True
        except subprocess.CalledProcessError as e:
            logging.error(f"rsync failed: {e.stderr}")
            return False
        except FileNotFoundError:
            logging.error("rsync command not found. Please install rsync.")
            return False
    
    def _copy_with_scp(self, host: str, remote_path: str, local_target: Path) -> bool:
        """
        Copy files using scp
        
        Args:
            host: Remote host address
            remote_path: Remote path
            local_target: Local destination path
            
        Returns:
            True on success
        """
        remote = self.build_remote_path(host, remote_path)
        
        # Build scp command
        # -r: Recursive copy
        # -p: Preserve timestamps and permissions
        cmd = [
            "scp",
            "-r",
            "-p",
            f"{remote}",
            str(local_target.parent)
        ]
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True
            )
            if result.stdout:
                logging.debug(result.stdout)
            
            # scp copies to parent directory, so adjust the name
            copied_dir = local_target.parent / Path(remote_path).name
            if copied_dir.exists() and copied_dir != local_target:
                if local_target.exists():
                    shutil.rmtree(local_target)
                copied_dir.rename(local_target)
            
            return True
        except subprocess.CalledProcessError as e:
            logging.error(f"scp failed: {e.stderr}")
            return False
        except FileNotFoundError:
            logging.error("scp command not found. Please install openssh-client.")
            return False
    
    def compress_backup(self) -> bool:
        """
        Compress backup folder to ZIP
        
        Returns:
            True on success
        """
        zip_path = Path(f"{self.backup_dir}.zip")
        logging.info(f"Compressing backup to {zip_path}...")
        
        try:
            with ZipFile(zip_path, 'w') as zipf:
                for file_path in self.backup_dir.rglob('*'):
                    if file_path.is_file():
                        # Archive path is relative to backup directory parent
                        arcname = file_path.relative_to(self.backup_dir.parent)
                        zipf.write(file_path, arcname)
            
            logging.info(f"Successfully created {zip_path}")
            
            # Remove original directory
            logging.info(f"Removing original directory {self.backup_dir}...")
            shutil.rmtree(self.backup_dir)
            logging.info("Original directory removed")
            
            return True
        except Exception as e:
            logging.error(f"Failed to compress backup: {e}")
            return False
    
    def run(self, ecu_list: list = None) -> bool:
        """
        Execute backup process
        
        Args:
            ecu_list: List of ECUs to backup (["ecu0"], ["ecu1"], or ["ecu0", "ecu1"]).
                     If None, backup both ECUs.
        
        Returns:
            True on success
        """
        logging.info(f"Starting DRS backup to {self.backup_dir}")
        
        # Create backup directory
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        
        # Determine which ECUs to backup
        if ecu_list is None:
            ecu_list = ["ecu0", "ecu1"]
        
        # Backup from specified ECUs
        success = True
        if "ecu0" in ecu_list:
            success &= self.backup_from_host(self.ECU0_HOST, "ecu0")
        if "ecu1" in ecu_list:
            success &= self.backup_from_host(self.ECU1_HOST, "ecu1")
        
        if not success:
            logging.error("Some backup operations failed")
            return False
        
        # Compress to ZIP
        if not self.compress_backup():
            logging.error("Failed to compress backup")
            return False
        
        logging.info("Backup completed successfully")
        return True


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        prog="backup_drs_config",
        description="Backup DRS configuration files from ecu0 and ecu1"
    )
    parser.add_argument(
        "-k", "--keyword",
        default=DRSBackup.DEFAULT_KEYWORD,
        help=f"Keyword for backup folder name (default: {DRSBackup.DEFAULT_KEYWORD})"
    )
    parser.add_argument(
        "-u", "--user",
        default=None,
        help="SSH username (default: current user)"
    )
    parser.add_argument(
        "--scp",
        action="store_true",
        help="Use scp instead of rsync (default: rsync)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--ecu",
        choices=["ecu0", "ecu1", "both"],
        default="both",
        help="ECU to backup: ecu0, ecu1, or both (default: both)"
    )
    
    args = parser.parse_args()
    
    backup = DRSBackup(
        keyword=args.keyword,
        user=args.user,
        use_scp=args.scp
    )
    backup.setup_logging(verbose=args.verbose)
    
    # Determine ECU list based on argument
    if args.ecu == "both":
        ecu_list = ["ecu0", "ecu1"]
    else:
        ecu_list = [args.ecu]
    
    success = backup.run(ecu_list=ecu_list)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

