import csv
import os
from datetime import datetime

class CSVRollingLogger:
    def __init__(self, directory, prefix, max_lines=1000):
        self.directory = directory
        self.prefix = prefix
        self.max_lines = max_lines
        self.current_file = None
        self.line_count = 0
        
        if not os.path.exists(directory):
            os.makedirs(directory)

    def _get_new_filepath(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.directory, f"{self.prefix}_{timestamp}.csv")

    def log_row(self, data_list):
        # Create new file if limit reached or first run [cite: 771]
        if self.current_file is None or self.line_count >= self.max_lines:
            self.current_file = self._get_new_filepath()
            self.line_count = 0
            # Write header if needed here
            
        with open(self.current_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(data_list)
            f.flush() # Ensure data is saved even if power is lost [cite: 772]
            self.line_count += 1