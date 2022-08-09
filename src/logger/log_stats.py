#!/usr/bin/env python2
# encoding: utf-8

import sys


def main():
    if len(sys.argv) < 2 or sys.argv[1] == 'h':
        print('usage: log_stats.py  <log_filename>')
        sys.exit(1)
    token_dict = {}
    log_filename = sys.argv[1]
    with open(log_filename) as f:
        for line in f:
            if line[0] == '#':
                continue
            tokens = line.split()
            token = tokens[0]
            ts = tokens[-1]
            if token == '':
                continue
            if token in token_dict:
                (count, ts_init, ts_end) = token_dict[token]
                count += 1
                ts_end = ts
                token_dict[token] = (count, ts_init, ts_end)
            else:
                token_dict[token] = (1, ts, ts)
    for token, (count, ts_init, ts_end) in sorted(token_dict.items()):
        try:
            Hz = float(count) / (float(ts_end) - float(ts_init))
        except ZeroDivisionError:
            Hz = 'n/a'
        print('{}:  count: {},  {:.2f} Hz,  initial timestamp: {},  final timestamp: {}'.format(token, count, Hz, ts_init, ts_end))
    

if __name__ == '__main__':
    main()
