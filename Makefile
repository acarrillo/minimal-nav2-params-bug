IMAGE_NAME = nav2-bug-repro

.PHONY: build-docker
build-docker:
	docker build -t $(IMAGE_NAME) .

.PHONY: shell
shell:
	docker run -it --rm \
		-v $(PWD):/workspace \
		-p 8765:8765 \
		--privileged \
		$(IMAGE_NAME) /bin/bash

.PHONY: build
build:
	colcon build --packages-select nav2_bug_repro_pkg

.PHONY: clean
clean:
	rm -rf log/ build/ install/