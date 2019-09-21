
# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

function rosbag_fancy() {
	rosrun rosbag_fancy rosbag_fancy $*
}

function _rosbag_fancy() {
	local cur="${COMP_WORDS[COMP_CWORD]}"
	local cmd="${COMP_WORDS[1]}"

	if [[ "$COMP_CWORD" == "1" ]]; then
		COMPREPLY=( $(compgen -W "record --help") )
		return
	fi

	case "${cmd}" in
		record)
			local FLAGS=( --help )
			local OPTS=( --topic --queue-size -o --output -p --prefix )

			# Are we currently inside an option?
			if [[ " ${OPTS[@]} " =~ " ${COMP_WORDS[COMP_CWORD-1]} " ]]; then
				case "${COMP_WORDS[COMP_CWORD-1]}" in
					--topic)
						COMPREPLY=( $(compgen -o nospace -W "$(rostopic list 2>/dev/null)" -- $cur) )
						compopt -o nospace
						;;
					-o|--output|-p|--prefix)
						COMPREPLY=( $(compgen -f -- $cur) )
						;;
					*)
						COMPREPLY=()
						;;
				esac
				return
			fi

			COMPREPLY=( $(compgen -o nospace -W "${FLAGS[*]} ${OPTS[*]} $(rostopic list 2>/dev/null)" -- $cur) )
			compopt -o nospace
			;;
	esac
}
complete -F _rosbag_fancy rosbag_fancy

