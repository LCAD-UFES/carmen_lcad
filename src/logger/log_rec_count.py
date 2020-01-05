import sys

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print "Usage:\n", sys.argv[0],'<logfile>'
        exit(0)
    
    rec_count = 0
    tag_count = {}
    logfilename = sys.argv[1]
    logfile = open(logfilename)

    for rec in logfile:
        rec_count += 1
        if rec_count % 1000 == 0:
            print 'rec count =', rec_count
        tag = rec.split()[0]
        if tag in tag_count:
            tag_count[tag] = tag_count.get(tag) + 1
        else:
            tag_count[tag] = 1

    logfile.close()
    
    print 'rec count =', rec_count
    for item in tag_count.items():
        print item[0], item[1]
