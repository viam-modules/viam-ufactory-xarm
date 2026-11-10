BIN_OUTPUT_PATH = bin
TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)
PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"
UNAME_S ?= $(shell uname -s)
UNAME_M ?= $(shell uname -m)

build:
	rm -rf bin
	go build -o $(BIN_OUTPUT_PATH)/viam-xarm

module: build
	rm -f $(BIN_OUTPUT_PATH)/module.tar.gz
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz $(BIN_OUTPUT_PATH)/viam-xarm meta.json

clean:
	rm -rf $(BIN_OUTPUT_PATH)/viam-xarm $(BIN_OUTPUT_PATH)/module.tar.gz

# Everything builds with the Go the module declares. Left to itself, go install of a tool picks
# the newest Go release that satisfies the tool, which on the day of a point release is one the
# module proxy does not serve yet, and a newer Go on a developer's machine builds the module
# with export data the linter, built with the declared Go, cannot read.
export GOTOOLCHAIN = go$(shell go list -m -f '{{.GoVersion}}')

tool-install:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/rhysd/actionlint/cmd/actionlint
	GOBIN=`pwd`/$(TOOL_BIN) go install github.com/golangci/golangci-lint/v2/cmd/golangci-lint@v2.14.0

gofmt:
	gofmt -w -s .

lint: gofmt tool-install
	go mod tidy
	PATH=$(PATH_WITH_TOOLS) golangci-lint run -c etc/.golangci.yaml --fix

update-rdk:
	go get go.viam.com/rdk@latest
	go mod tidy

test: tool-install
	go test -v -race -failfast ./...
