#!/usr/bin/env python2
# encoding: utf-8

import sys


def main():
    if len(sys.argv) < 2 or sys.argv[1] == '-h':
        print('\nUSAGE:\n{}  <log_filename>\n'.format(sys.argv[0]))
        sys.exit(1)
    log_filename = sys.argv[1]
    token_dict = {}
    try:
        with open(log_filename) as f:
            for line in f:
                tokens = line.split()
                if not tokens or tokens[0] == '#':
                    continue
                token = tokens[0]
                ts = tokens[-1]
                if token in token_dict:
                    (count, ts_init, ts_end) = token_dict[token]
                    token_dict[token] = (count + 1, ts_init, ts)
                else:
                    token_dict[token] = (1, ts, ts)
    except IOError:
        print('\nERROR: Could not open log filename: {}\n'.format(log_filename))
        sys.exit(2)
    print
    total_count = 0
    for token, (count, ts_init, ts_end) in sorted(token_dict.items()):
        total_count += count
        try:
            Hz = float(count - 1) / (float(ts_end) - float(ts_init))
        except ZeroDivisionError:
            Hz = 0.0
        print('{}:  count: {},  {:.2f} Hz,  initial timestamp: {},  final timestamp: {}'.format(token, count, Hz, ts_init, ts_end))
    print('\nFile: {},  message types: {},  total message count: {}\n'.format(log_filename, len(token_dict), total_count))
    

if __name__ == '__main__':
    main()
