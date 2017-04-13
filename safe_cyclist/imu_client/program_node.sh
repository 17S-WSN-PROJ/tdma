if [ $# -ne 2 ]; then
   echo "Pass in a mac with -m"
fi
if[ $1 -ne "-m"]; then
  echo "Pass in a mac with -m"
fi



function usage {
   cat << EOF
   Usage: program_node.sh -m <application>

   Performs some activity
   EOF
      exit 1
}
