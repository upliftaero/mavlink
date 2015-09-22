__author__ = 'kwells'

list_of_dicts =
out_path= "flights.csv"

fieldnames = sorted(list(set(k for d in list_of_dicts for k in d)))

with open(out_path, 'wb') as out_file:
    writer = csv.DictWriter(out_file, fieldnames=fieldnames, dialect='excel')
    writer.writeheader()
    writer.writerows(list_of_dicts)