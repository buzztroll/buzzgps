SUBDIRS = src examples tests


if ENABLE_COVERAGE

cov-reset:
	@rm -fr coverage
	@find . -name "*.gcda" -exec rm {} \;
	@lcov --directory . --zerocounters

cov-report:
	@mkdir -p coverage
	@lcov --compat-libtool --directory . --capture --output-file coverage/app.info  
	@genhtml -o coverage/ coverage/app.info 

cov:
	@make cov-report

clean-local:
	@make cov-reset 

check:
	@make cov
endif    
