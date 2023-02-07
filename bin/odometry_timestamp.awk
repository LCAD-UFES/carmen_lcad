BEGIN {
  previous_timestamp=0;
  first_timestamp=0
}
$1 == "ROBOTVELOCITY_ACK" {
  if (first_timestamp==0)
  {
     first_timestamp = $4
     previous_timestamp = first_timestamp
  }
  print $4-first_timestamp " " $4 - previous_timestamp;
  previous_timestamp=$4
}
END {
}

