#!/bin/bash
# utility script to build the ardupilot_wiki with docker

BOLD='\e[1m'
RED='\e[31m'
GREEN='\e[32m'
YELLOW='\e[33m'
MAGENTA='\e[35m'
NC='\e[0m' # no color

show_menu() {
    echo -e "\n${BOLD}${YELLOW}Select ArduPilot Wiki site to build:${NC}\n"
    echo -e "  ${MAGENTA}1) Copter${NC}"
    echo -e "  ${MAGENTA}2) Plane${NC}"
    echo -e "  ${MAGENTA}3) Rover${NC}"
    echo -e "  ${MAGENTA}4) Dev${NC}"
    echo -e "  ${MAGENTA}5) All${NC}"
    echo -en "\nEnter choice [1-5]: "
    read choice

    case $choice in
        1) site_option="--site copter" ;;
        2) site_option="--site plane" ;;
        3) site_option="--site rover" ;;
        4) site_option="--site dev" ;;
        5) site_option="" ;;
        *) echo -e "\n${BOLD}${RED}Invalid choice.${NC}" && show_menu ;;
    esac

    echo -en "\nBuild fast? (Y/n): "
    read fast_choice
    if [[ -z "$fast_choice" || "$fast_choice" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        fast_option="--fast"
    else
        fast_option=""
    fi

    echo -e "\nRunning: ${BOLD}${GREEN}update.py $fast_option $site_option${NC}\n"
    run_command $fast_option $site_option
}

run_command() {
    docker run --rm -it -v "${PWD}:/ardupilot_wiki" -u "$(id -u):$(id -g)" ardupilot_wiki python3 update.py "$@"
}

if [ "$#" -eq 0 ]; then
    # if no arguments given, show menu of options for building
    show_menu
else
    # build with docker using arguments passed via CLI
    run_command "$@"
fi
