
# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

# cache rosmon executable so that we don't need a rosrun invocation each
# time we do completion
_ROSMON_EXECUTABLE=$(rosrun --prefix echo rosbag_fancy rosbag_fancy)

function rosbag_fancy() {
	rosrun rosbag_fancy rosbag_fancy $*
}

function _rosbag_fancy() {
	local cur="${COMP_WORDS[COMP_CWORD]}"
	local cmd="${COMP_WORDS[1]}"

	local FLAGS=( --help )
	local OPTS=( --topic --queue-size -o --output )

	# Are we currently inside an option?
	if [[ " ${OPTS[@]} " =~ " ${COMP_WORDS[COMP_CWORD-1]} " ]]; then
		case "${COMP_WORDS[COMP_CWORD-1]}" in
			--topic)
				COMPREPLY=( $(compgen -o nospace -W "$(rostopic list)" -- $cur) )
				compopt -o nospace
				;;
			-o|--output)
				COMPREPLY=( $(compgen -f -- $cur) )
				;;
			*)
				COMPREPLY=()
				;;
		esac
		return
	fi

	COMPREPLY=( $(compgen -o nospace -W "${FLAGS[*]} ${OPTS[*]} $(rostopic list)" -- $cur) )
	compopt -o nospace
}
complete -F _rosbag_fancy rosbag_fancy
