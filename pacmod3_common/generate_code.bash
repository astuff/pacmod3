#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CODER_DBC_PATH="$HOME/git/coderdbc/build/coderdbc"
DBC_VERSION="$1"

main()
{
  DBC_MAJOR_VERSION="${DBC_VERSION%%.*}"
  DRIVER_NAME="pacmod$DBC_MAJOR_VERSION"

  TEMP_DIR="$(mktemp -d)"

  # TEMP_DIR=/tmp/testing_thing
  # mkdir -p "$TEMP_DIR"

  cd "$TEMP_DIR"
  mkdir -p output

  get_dbc

  # echo "$CODER_DBC_PATH"
  echo "Auto-generating code for version $DBC_VERSION of the PacMod DBC."
  echo "Generated code will use the name $DRIVER_NAME"
  echo ""

  "$CODER_DBC_PATH" -dbc "$TEMP_DIR/pacmod_dbc/as_pacmod.dbc" -out "$TEMP_DIR/output" -drvname "$DRIVER_NAME" -rw

  cp "$TEMP_DIR/output/lib/$DRIVER_NAME.c" "$SCRIPT_DIR/autogen/"
  cp "$TEMP_DIR/output/lib/$DRIVER_NAME.h" "$SCRIPT_DIR/autogen/"
  cp "$TEMP_DIR/output/conf/$DRIVER_NAME-config.h" "$SCRIPT_DIR/autogen/"
  cp "$TEMP_DIR/output/conf/dbccodeconf.h" "$SCRIPT_DIR/autogen/"

  echo ""
  echo "Done"
}

get_dbc()
{
  git clone https://github.com/astuff/pacmod_dbc.git
  cd pacmod_dbc
  git checkout "$DBC_VERSION"
}

main
