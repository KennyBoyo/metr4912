#!/bin/bash

USER_ID='JunKhai'
USER_PW='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'

origin=$(git remote get-url origin)
origin_with_pass=${origin/"//"/"//${USER_ID}:${USER_PW}@"}
git push ${origin_with_pass} master
