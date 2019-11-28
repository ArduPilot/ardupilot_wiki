#!/bin/bash

#===========================================================================================
#	Automatically add last modified date to .RST files according 
#	to last commit date of each file.
#
#   Caution: do use while updating the Wiki in the server because it alters ALL .RST files.
#
#	Run: ./insert_last_updated_date.sh N (where N is the number is the parallel tasks
#
#===========================================================================================

# Parallel runs in N-process batches. Default of $1 is 2 
PARALLEL=${1:-2}

# Insert last updated date from last commit date of each file
update_file(){

	commit_date=`git log -n 1 --date=short --pretty=format:%cd -- $1`

	if [ "$commit_date" != "" ]
	then
		echo "Inserting $commit_date as last updated tag on $f"
		echo "" >> $1
		echo "" >> $1
		echo '*Page last edited on ' "$commit_date"'*' >> $1		
	else
		echo "Git last commit date not found for $1"
	fi
	
}

# Runs for each file at PARALLEL at time 
for f in `find . -iname "*.rst"`; do

   ((i=i%PARALLEL)); ((i++==0)) && wait
   update_file $f & 
		
done

echo 'Finished to insert last edited date.'

exit 0

