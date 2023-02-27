#!/bin/bash

USER_ID='JunKhai'
USER_PW='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'

origin=$(git remote get-url origin)
origin_with_pass=${origin/"//"/"//${USER_ID}:${USER_PW}@"}
git push ${origin_with_pass} master
