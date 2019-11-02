#!/usr/bin/python

import unittest

class TestSample(unittest.TestCase):

    # @unittest.skip("skip sample test")
    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FOO')

if __name__ == '__main__':
    unittest.main()