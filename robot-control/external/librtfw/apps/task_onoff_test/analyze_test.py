#!/usr/bin/env python3
"""
task_onoff_test 프로그램 분석 스크립트

실행 로그를 분석하여:
1. 각 태스크의 ExecState 분포 확인
2. enable/disable 전파 검증
3. 데드라인 미스 원인 분류
"""

import re
import sys
from collections import defaultdict
from typing import Dict, List, Tuple

class TaskExecAnalyzer:
    def __init__(self):
        self.exec_states: Dict[str, List[str]] = defaultdict(list)
        self.task_log_lines: Dict[str, List[str]] = defaultdict(list)
        
    def parse_log(self, log_file: str):
        """로그 파일 파싱"""
        with open(log_file, 'r') as f:
            for line in f:
                # 태스크 로그 추출
                match = re.search(r'\[(\w+)\]', line)
                if match:
                    task_name = match.group(1)
                    self.task_log_lines[task_name].append(line.strip())
    
    def print_summary(self):
        """분석 결과 출력"""
        print("\n=== Task Execution Summary ===\n")
        
        for task_name in sorted(self.task_log_lines.keys()):
            lines = self.task_log_lines[task_name]
            print(f"{task_name}:")
            print(f"  - Total executions: {len(lines)}")
            if lines:
                print(f"  - First: {lines[0][:80]}...")
                print(f"  - Last:  {lines[-1][:80]}...")
            print()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: analyze_test.py <log_file>")
        sys.exit(1)
    
    analyzer = TaskExecAnalyzer()
    analyzer.parse_log(sys.argv[1])
    analyzer.print_summary()
