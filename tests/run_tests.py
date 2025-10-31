#!/usr/bin/env python3
"""
Simple test runner for OAMS tests.
Run with: python run_tests.py
"""

import sys
import os

# Add the tests directory to the path
sys.path.insert(0, os.path.dirname(__file__))

from test_oams_system import OAMSTestSuite

def main():
    """Run the OAMS test suite."""
    print("OAMS Test Runner")
    print("================")
    
    try:
        # Create and run test suite
        suite = OAMSTestSuite()
        suite.setUp()
        suite.run_all_tests()
        
        return 0
        
    except Exception as e:
        print(f"\nFatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(main())
